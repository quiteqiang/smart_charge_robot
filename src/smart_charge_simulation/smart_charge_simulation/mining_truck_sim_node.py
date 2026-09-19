#!/usr/bin/env python3
"""矿卡最小仿真器（rclpy）。

为什么自研而不是 Gazebo：目标机器仅 2GB 内存，Gazebo Harmonic + RViz + Nav2
无法同时驻留；本节点用解析 2D 光线投射替代物理引擎，确定性高、可无头测试。

发布：/odom、/imu、/scan、/joint_states、/dock_relative_pose（模拟 AprilTag
检测到的充电桩相对位姿）、TF: odom->base_link->base_laser/imu_link。
订阅：/cmd_vel（速度指令）、/sim/reset_pose（测试用重置）、/sim/obstacles
（动态障碍物，visualization_msgs/Marker，CUBE 增删）。
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import rclpy
from ament_index_python.packages import get_package_share_directory

from smart_charge_simulation.urdf_kinematics import parse_kinematics
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker

import yaml

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    # 瘦主机可只装 rosdep 最小集：无 numpy 时激光回退纯 Python 逐条求交（原实现）
    np = None
    _HAS_NUMPY = False


@dataclass
class Rect:
    xmin: float
    ymin: float
    xmax: float
    ymax: float


@dataclass
class World:
    rects: list[Rect] = field(default_factory=list)
    pillars: list[tuple[float, float, float]] = field(default_factory=list)

    def raycast(self, ox: float, oy: float, dx: float, dy: float, max_range: float) -> float:
        """返回沿单位方向 (dx,dy) 的最近障碍物距离，未命中返回 max_range。"""
        best = max_range
        for r in self.rects:
            t = ray_rect(ox, oy, dx, dy, r)
            if t is not None and t < best:
                best = t
        for cx, cy, rad in self.pillars:
            t = ray_circle(ox, oy, dx, dy, cx, cy, rad)
            if t is not None and t < best:
                best = t
        return best


def ray_rect(ox: float, oy: float, dx: float, dy: float, r: Rect) -> float | None:
    """slab 法射线-AABB 相交，返回最近正交距离。"""
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, r.xmin, r.xmax), (oy, dy, r.ymin, r.ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    if tmax < 0.0:
        return None
    return tmin if tmin > 0 else tmax


def ray_circle(ox: float, oy: float, dx: float, dy: float,
               cx: float, cy: float, rad: float) -> float | None:
    lx, ly = cx - ox, cy - oy
    tca = lx * dx + ly * dy
    d2 = lx * lx + ly * ly - tca * tca
    r2 = rad * rad
    if d2 > r2:
        return None
    thc = math.sqrt(r2 - d2)
    t0 = tca - thc
    t1 = tca + thc
    if t0 > 0:
        return t0
    if t1 > 0:  # 起点在圆内
        return t1
    return None


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def raycast_static_np(rects: 'np.ndarray', pillars: 'np.ndarray',
                      ox: float, oy: float,
                      dx: 'np.ndarray', dy: 'np.ndarray',
                      r_max: float) -> 'np.ndarray':
    """静态世界向量化求交：dx/dy 为 (B,) 光束单位方向，返回 (B,) 最近距离。

    与 ray_rect/ray_circle 纯 Python 版逐点语义一致（对拍验证）：
    slab 法处理 AABB，tmin<=0 时取出口 tmax，平行且原点在板外视为无交。
    180 beam × 全量 rect/pillar 由 Python 层 ~千次调用压成几次数组运算。
    """
    neg_inf, inf = -math.inf, math.inf
    best = np.full(dx.shape[0], r_max)
    if rects.shape[0]:
        xmin, xmax = rects[:, 0][None, :], rects[:, 1][None, :]
        ymin, ymax = rects[:, 2][None, :], rects[:, 3][None, :]
        dxr, dyr = dx[:, None], dy[:, None]
        eps = 1e-12
        par_x, par_y = np.abs(dxr) < eps, np.abs(dyr) < eps
        with np.errstate(divide='ignore', invalid='ignore'):
            t1x, t2x = (xmin - ox) / dxr, (xmax - ox) / dxr
            t1y, t2y = (ymin - oy) / dyr, (ymax - oy) / dyr
        tminx = np.where(par_x, neg_inf, np.minimum(t1x, t2x))
        tmaxx = np.where(par_x, inf, np.maximum(t1x, t2x))
        tminy = np.where(par_y, neg_inf, np.minimum(t1y, t2y))
        tmaxy = np.where(par_y, inf, np.maximum(t1y, t2y))
        bad = (par_x & ((ox < xmin) | (ox > xmax))) | (par_y & ((oy < ymin) | (oy > ymax)))
        tmin = np.maximum(tminx, tminy)
        tmax = np.minimum(tmaxx, tmaxy)
        valid = ~bad & (tmin <= tmax) & (tmax >= 0.0)
        t = np.where(tmin > 0.0, tmin, tmax)   # 起点在盒内时取出口距离
        best = np.minimum(best, np.where(valid, t, inf).min(axis=1))
    if pillars.shape[0]:
        cx, cy, rad = pillars[:, 0][None, :], pillars[:, 1][None, :], pillars[:, 2][None, :]
        lx, ly = cx - ox, cy - oy
        tca = lx * dx[:, None] + ly * dy[:, None]
        d2 = lx * lx + ly * ly - tca * tca
        thc = np.sqrt(np.maximum(rad * rad - d2, 0.0))
        t0, t1 = tca - thc, tca + thc
        t = np.where(t0 > 0.0, t0, np.where(t1 > 0.0, t1, inf))
        best = np.minimum(best, np.where(d2 <= rad * rad, t, inf).min(axis=1))
    return best


class MiningTruckSim(Node):
    def __init__(self) -> None:
        super().__init__('mining_truck_sim')

        self.declare_parameter('world_file', '')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('scan_beams', 180)
        self.declare_parameter('scan_rate_hz', 8.0)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('cmd_timeout_s', 0.6)
        self.declare_parameter('max_linear_vel', 0.6)
        self.declare_parameter('max_angular_vel', 1.2)
        # wheel_radius / wheel_separation / laser_x 的单一事实源是
        # smart_charge_base 的 mining_truck.urdf（improvement_directions #8）；
        # 下列参数仅为 URDF 缺失/不可解析时的 fallback 默认值
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheel_separation', 0.42)
        self.declare_parameter('laser_x', 0.15)   # base_laser 在 base_link 中的安装位置
        self.declare_parameter('urdf_file', '')   # 空 = 解析 smart_charge_base 包内 URDF
        self.declare_parameter('publish_ground_truth_pose', True)
        self.declare_parameter('ground_truth_frame', 'map')

        world_file = self.get_parameter('world_file').get_parameter_value().string_value
        with open(world_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.static_world = World(
            rects=[Rect(float(r['xmin']), float(r['ymin']), float(r['xmax']), float(r['ymax']))
                   for r in data['walls'] + data['obstacles']],
            pillars=[(float(p['x']), float(p['y']), float(p['r'])) for p in data['pillars']],
        )
        self.dock_pose = (float(data['dock_pose']['x']),
                          float(data['dock_pose']['y']),
                          float(data['dock_pose']['yaw']))
        start = data.get('start_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

        # 机器人状态（真值）
        self.x = float(start['x'])
        self.y = float(start['y'])
        self.yaw = float(start['yaw'])
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.wheel_l = 0.0
        self.wheel_r = 0.0
        self._clock_now = None
        self.dynamic_world = World()   # /sim/obstacles 注入的动态障碍物

        # ---- 激光静态缓存（参数与安装位置运行时不变；光束角度固定） ----
        self._scan_beams = int(self.get_parameter('scan_beams').value)
        self._scan_range_min = float(self.get_parameter('range_min').value)
        self._scan_range_max = float(self.get_parameter('range_max').value)
        self._laser_x = float(self.get_parameter('laser_x').value)
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._wheel_separation = float(self.get_parameter('wheel_separation').value)
        self._resolve_kinematics_from_urdf()
        self._np = np if _HAS_NUMPY else None
        if self._np is not None:
            self._static_rects_np = self._np.array(
                [[r.xmin, r.xmax, r.ymin, r.ymax] for r in self.static_world.rects],
                dtype=float).reshape(-1, 4)
            self._static_pillars_np = self._np.array(
                list(self.static_world.pillars), dtype=float).reshape(-1, 3)
            angles = self._np.linspace(-math.pi, math.pi, self._scan_beams, endpoint=False)
            self._beam_cos = self._np.cos(angles)
            self._beam_sin = self._np.sin(angles)
        # 这四个参数已缓存为成员：ros2 param set 后经此回调立即生效，
        # 避免"set 成功但扫描仍用旧值"的假象
        self.add_on_set_parameters_callback(self._on_set_scan_params)

        # ---- 接口 ----
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(PoseStamped, '/sim/reset_pose', self._on_reset, 10)
        self.create_subscription(Marker, '/sim/obstacles', self._on_obstacle_marker, 10)
        self.create_subscription(Bool, '/sim/dock_visible', self._on_dock_visible, 10)
        self._dock_visible = True

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.dock_pub = self.create_publisher(PoseStamped, '/dock_relative_pose', 10)
        self.dock_contact_pub = self.create_publisher(Bool, '/dock_contact', 10)
        self.dock_marker_pub = self.create_publisher(Marker, '/visualization_marker_dock', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        rate = self.get_parameter('rate_hz').value
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(1.0 / rate, self._on_tick)
        self.create_timer(1.0 / self.get_parameter('scan_rate_hz').value, self._publish_scan)      # 8 Hz 激光
        self.create_timer(1.0 / 10.0, self._publish_dock_pose)  # 10 Hz 充电桩相对位姿（泊靠控制器 pose_timeout_s=1.0 的裕度）
        self.create_timer(1.0, self._publish_dock_marker)
        self.get_logger().info(f'矿卡仿真器就绪，起始位姿 ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})')

    # ---------- 回调 ----------
    def _resolve_kinematics_from_urdf(self) -> None:
        """URDF 为运动学参数单一事实源（#8）；失败则保留 yaml fallback。"""
        urdf_file = self.get_parameter('urdf_file').value
        if not urdf_file:
            try:
                urdf_file = os.path.join(
                    get_package_share_directory('smart_charge_base'),
                    'urdf', 'mining_truck.urdf')
            except Exception:  # noqa: BLE001 - 包不可定位时走 fallback
                urdf_file = ''
        kin = None
        if urdf_file and os.path.isfile(urdf_file):
            try:
                with open(urdf_file, 'r', encoding='utf-8') as f:
                    kin = parse_kinematics(f.read())
            except OSError:
                kin = None
        self._kinematics_from_urdf = kin is not None
        if kin is None:
            self.get_logger().warn(
                f'URDF 运动学参数不可用（{urdf_file or "未找到文件"}），'
                '回退 sim_params.yaml 的 wheel_radius/wheel_separation/laser_x')
            return
        self._wheel_radius = kin.wheel_radius
        self._wheel_separation = kin.wheel_separation
        self._laser_x = kin.laser_x
        self.get_logger().info(
            '运动学参数来自 URDF（单一事实源）: '
            f'wheel_radius={self._wheel_radius}, '
            f'wheel_separation={self._wheel_separation}, '
            f'laser_x={self._laser_x}')

    def _wheel_kinematics(self) -> tuple[float, float]:
        """(wheel_separation, wheel_radius)。

        URDF 模式用启动时缓存（单一事实源）；fallback 模式每次实时读参，
        保留 ros2 param set 在线标定能力（review finding 5）。
        """
        if self._kinematics_from_urdf:
            return self._wheel_separation, self._wheel_radius
        return (float(self.get_parameter('wheel_separation').value),
                float(self.get_parameter('wheel_radius').value))

    def _on_set_scan_params(self, params):
        """缓存型激光参数（scan_beams/range_min/range_max）运行时更新。

        kinematics 参数（laser_x/wheel_radius/wheel_separation）在 URDF
        模式下拒绝运行时覆盖：TF 由 robot_state_publisher 按 URDF 发布，
        单方面移动 raycast 原点会使 scan 与其坐标系脱节（review finding 4）；
        fallback 模式下 laser_x 仍可改，轮参数由 _wheel_kinematics 实时读取。
        """
        result = SetParametersResult()
        result.successful = True
        result.reason = ''
        for p in params:
            if p.type_ == Parameter.Type.NOT_SET:
                continue
            if p.name == 'scan_beams':
                if p.type_ != Parameter.Type.INTEGER or p.value < 1:
                    result.successful = False
                    result.reason = 'scan_beams 必须为正整数'
                    continue
                self._scan_beams = p.value
                if self._np is not None:
                    angles = self._np.linspace(-math.pi, math.pi, self._scan_beams, endpoint=False)
                    self._beam_cos = self._np.cos(angles)
                    self._beam_sin = self._np.sin(angles)
            elif p.name in ('range_min', 'range_max', 'laser_x'):
                if p.type_ != Parameter.Type.DOUBLE:
                    result.successful = False
                    result.reason = f'{p.name} 必须为浮点数'
                    continue
                if p.name == 'range_min':
                    self._scan_range_min = p.value
                elif p.name == 'range_max':
                    self._scan_range_max = p.value
                elif self._kinematics_from_urdf:
                    result.successful = False
                    result.reason = ('laser_x 单一事实源为 URDF（#8）：运行时覆盖'
                                     '会使 scan 与 base_laser TF 脱节')
                    continue
                else:
                    self._laser_x = p.value
            elif p.name in ('wheel_radius', 'wheel_separation'):
                # URDF 模式：单一事实源，拒绝 silent no-op；fallback 模式
                # 每次 tick 实时读取（_wheel_kinematics），此处无需动作
                if self._kinematics_from_urdf:
                    result.successful = False
                    result.reason = f'{p.name} 单一事实源为 URDF（#8）'
                    continue
        return result

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _on_reset(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.x, self.y, self.yaw = msg.pose.position.x, msg.pose.position.y, yaw
        self.cmd_v = self.cmd_w = 0.0
        self.get_logger().warn(f'仿真位姿被重置为 ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})')

    def _on_obstacle_marker(self, msg: Marker) -> None:
        """测试场景 2 用：向世界中增删一个动态矩形障碍物。"""
        if msg.action == Marker.DELETE or msg.action == Marker.DELETEALL:
            self.dynamic_world.rects.clear()
            self.get_logger().warn('动态障碍物已移除')
            return
        s = msg.scale
        if s.x <= 0 or s.y <= 0:
            return
        rect = Rect(msg.pose.position.x - s.x / 2, msg.pose.position.y - s.y / 2,
                    msg.pose.position.x + s.x / 2, msg.pose.position.y + s.y / 2)
        self.dynamic_world.rects.append(rect)
        self.get_logger().warn(f'动态障碍物加入: {rect}')

    def _on_dock_visible(self, msg: Bool) -> None:
        """测试用：模拟 AprilTag 感知丢失（桩标记不可见）。"""
        if msg.data != self._dock_visible:
            self.get_logger().warn(f'充电桩标记可见性: {self._dock_visible} -> {msg.data}')
        self._dock_visible = msg.data

    # ---------- 主循环 ----------
    def _on_tick(self) -> None:
        now = self.get_clock().now()
        dt = 0.0 if self._clock_now is None else (now - self._clock_now).nanoseconds * 1e-9
        self._clock_now = now
        if dt <= 0.0 or dt > 0.5:
            return

        # 指令超时保护：仿真器层面也必须有，避免控制节点崩溃后机器人飞车
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.get_parameter('cmd_timeout_s').value:
            v, w = 0.0, 0.0
        else:
            v = max(-self.get_parameter('max_linear_vel').value,
                    min(self.get_parameter('max_linear_vel').value, self.cmd_v))
            w = max(-self.get_parameter('max_angular_vel').value,
                    min(self.get_parameter('max_angular_vel').value, self.cmd_w))

        # 差速模型积分
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + w * dt)
        sep, radius = self._wheel_kinematics()
        self.wheel_l += (v - w * sep / 2.0) / radius * dt
        self.wheel_r += (v + w * sep / 2.0) / radius * dt

        stamp = now.to_msg()
        self._publish_odom(stamp, v, w)
        self._publish_imu(stamp, w)
        self._publish_joint_states(stamp)
        self._publish_tf(stamp)

    def _publish_odom(self, stamp, v: float, w: float) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.odom_pub.publish(msg)

    def _publish_imu(self, stamp, w: float) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        msg.orientation.x, msg.orientation.y = q[0], q[1]
        msg.orientation.z, msg.orientation.w = q[2], q[3]
        msg.angular_velocity.z = w
        self.imu_pub.publish(msg)

    def _publish_joint_states(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.wheel_l, self.wheel_r]
        self.joint_pub.publish(msg)

    def _publish_tf(self, stamp) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.tf_broadcaster.sendTransform(t)

        # base_link -> base_laser / imu_link 由 robot_state_publisher(URDF) 发布

    # ---------- 激光雷达：解析光线投射 ----------
    def _publish_scan(self) -> None:
        beams = self._scan_beams
        r_min = self._scan_range_min
        r_max = self._scan_range_max
        lx = self.x + self._laser_x * math.cos(self.yaw)
        ly = self.y + self._laser_x * math.sin(self.yaw)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_laser'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / beams
        msg.range_min = r_min
        msg.range_max = r_max

        if self._np is not None:
            # numpy 路径：静态世界整块向量化求交；动态障碍物（通常为空或个位数）
            # 仍走纯 Python 逐条，避免为小规模数据付广播开销
            npx = self._np
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx = self._beam_cos * c - self._beam_sin * s
            dy = self._beam_sin * c + self._beam_cos * s
            ranges = raycast_static_np(self._static_rects_np, self._static_pillars_np,
                                       lx, ly, dx, dy, r_max)
            if self.dynamic_world.rects or self.dynamic_world.pillars:
                for i in range(beams):
                    r_dyn = self.dynamic_world.raycast(lx, ly, float(dx[i]), float(dy[i]), r_max)
                    if r_dyn < ranges[i]:
                        ranges[i] = r_dyn
            msg.ranges = npx.where(ranges >= r_min, ranges, npx.inf).tolist()
        else:
            # 语义：未命中为 inf（0.0 是非法值且会被消费者当近障碍）
            msg.ranges = [float('inf')] * beams
            for i in range(beams):
                a = msg.angle_min + i * msg.angle_increment
                dx, dy = math.cos(self.yaw + a), math.sin(self.yaw + a)
                r = self.static_world.raycast(lx, ly, dx, dy, r_max)
                r_dyn = self.dynamic_world.raycast(lx, ly, dx, dy, r_max)
                r = min(r, r_dyn)
                msg.ranges[i] = r if r >= r_min else float('inf')
        self.scan_pub.publish(msg)

    # ---------- 模拟 AprilTag：充电桩在 base_link 系下的位姿 ----------
    def _publish_dock_pose(self) -> None:
        if not self._dock_visible:
            return  # 模拟感知丢失：不再发布相对位姿
        dx = self.dock_pose[0] - self.x
        dy = self.dock_pose[1] - self.y
        # 转到 base_link 系
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        rx = dx * c - dy * s
        ry = dx * s + dy * c
        ryaw = normalize_angle(self.dock_pose[2] - self.yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = rx
        msg.pose.position.y = ry
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, ryaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.dock_pub.publish(msg)

        # 物理接触判定：机器人中心距充电桩足够近 => 充电枪可连接。
        # 电池节点仅在 /dock_contact 与 /charging_active(任务机请求) 同时为真时充电。
        dist = math.hypot(self.dock_pose[0] - self.x, self.dock_pose[1] - self.y)
        self.dock_contact_pub.publish(Bool(data=dist < 0.35))

    def _publish_dock_marker(self) -> None:
        """RViz 充电桩可视化（贴墙安装块）。"""
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.ns = 'dock'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = self.dock_pose[0] - 0.15
        m.pose.position.y = self.dock_pose[1]
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = 1.0
        m.scale.y = 1.5
        m.scale.z = 0.6
        m.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.8)
        m.lifetime.sec = 2
        self.dock_marker_pub.publish(m)


def main() -> None:
    rclpy.init()
    node = MiningTruckSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
