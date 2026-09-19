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
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker

import yaml


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
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheel_separation', 0.42)
        self.declare_parameter('laser_x', 0.15)   # base_laser 在 base_link 中的安装位置
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
        sep = self.get_parameter('wheel_separation').value
        radius = self.get_parameter('wheel_radius').value
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
        beams = int(self.get_parameter('scan_beams').value)
        r_min = self.get_parameter('range_min').value
        r_max = self.get_parameter('range_max').value
        lx = self.x + self.get_parameter('laser_x').value * math.cos(self.yaw)
        ly = self.y + self.get_parameter('laser_x').value * math.sin(self.yaw)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_laser'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / beams
        msg.range_min = r_min
        msg.range_max = r_max
        msg.ranges = [0.0] * beams
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
