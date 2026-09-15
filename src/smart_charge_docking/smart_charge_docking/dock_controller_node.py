#!/usr/bin/env python3
"""精准泊靠控制器（Docking Server 的最小可行替代）。

为什么不用 opennav_docking：最小仿真器无相机/AprilTag 检测栈，Docking Server
的检测插件在此环境属于依赖不兼容；本节点契约与 Docking Server 对齐
（相对位姿输入 + 泊靠/离桩服务 + 结果上报），后续可平滑迁移。

输入：/dock_relative_pose（充电桩在 base_link 系下的位姿，模拟 AprilTag 输出）
输出：低速 /cmd_vel（仅泊靠期间发布，Nav2 目标结束后不存在竞争）
服务：/dock/start（std_srvs/Trigger）、/dock/undock
上报：/docking_success（Bool，latch）、/docking_status（String）

控制序列：朝向对齐（原地旋转）→ 接近（比例控制 + 越近越慢）→ 精调（低速小步）
安全：限速、超时、前向激光碰撞急停；重试由 mission 状态机负责（最多 3 次）。
"""
from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class DockController(Node):
    ST_IDLE = 'IDLE'
    ST_ALIGN = 'ALIGN'          # 原地朝向对齐
    ST_APPROACH = 'APPROACH'    # 接近
    ST_FINAL = 'FINAL'          # 低速精调
    ST_REVERSE = 'REVERSE'      # 离桩倒车
    ST_DONE = 'DONE'
    ST_FAILED = 'FAILED'

    def __init__(self) -> None:
        super().__init__('dock_controller')

        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('xy_tolerance', 0.04)
        self.declare_parameter('yaw_tolerance', 0.06)
        self.declare_parameter('kp_yaw', 1.8)
        self.declare_parameter('kp_y', 1.2)
        self.declare_parameter('kp_dist', 0.8)
        self.declare_parameter('final_linear_cap', 0.05)       # 末段限速
        self.declare_parameter('final_range', 0.35)            # 进入末段的距离
        self.declare_parameter('align_tolerance', 0.15)        # 接近前允许的朝向误差
        self.declare_parameter('timeout_s', 45.0)
        self.declare_parameter('collision_stop_distance', 0.18)
        self.declare_parameter('undock_distance', 0.8)
        self.declare_parameter('undock_timeout_s', 30.0)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('pose_timeout_s', 1.0)

        self.state = self.ST_IDLE
        self.dock_rel: tuple[float, float, float] | None = None   # x, y, yaw(base 系)
        self.dock_rel_stamp = Time()
        self.front_min_range = float('inf')
        self.start_time = Time()
        self.start_x = 0.0

        self.create_subscription(PoseStamped, '/dock_relative_pose', self._on_dock_pose, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.success_pub = self.create_publisher(
            Bool, '/docking_success',
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(String, '/docking_status', 10)

        self.create_service(Trigger, '/dock/start', self._on_start)
        self.create_service(Trigger, '/dock/undock', self._on_undock)
        self.create_timer(1.0 / self.get_parameter('control_hz').value, self._on_control)
        self.success_pub.publish(Bool(data=False))

    # ---------- 接口 ----------
    def _on_dock_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.dock_rel = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.dock_rel_stamp = self.get_clock().now()

    def _on_scan(self, msg: LaserScan) -> None:
        # 前向 ±60° 最近距离，用于碰撞急停
        n = len(msg.ranges)
        front = []
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if abs(normalize_angle(ang)) < math.pi / 3 and math.isfinite(r):
                front.append(r)
        self.front_min_range = min(front) if front else float('inf')

    def _on_start(self, _, response: Trigger.Response) -> Trigger.Response:
        if self.state not in (self.ST_IDLE, self.ST_DONE, self.ST_FAILED):
            response.success = False
            response.message = f'泊靠忙: {self.state}'
            return response
        if self.dock_rel is None:
            response.success = False
            response.message = '未收到充电桩相对位姿'
            return response
        self.state = self.ST_ALIGN
        self.start_time = self.get_clock().now()
        self._report(f'开始泊靠, 相对位姿 x={self.dock_rel[0]:.2f} y={self.dock_rel[1]:.2f}')
        response.success = True
        response.message = 'docking started'
        return response

    def _on_undock(self, _, response: Trigger.Response) -> Trigger.Response:
        if self.state != self.ST_DONE:
            response.success = False
            response.message = f'不在已泊靠状态: {self.state}'
            return response
        self.state = self.ST_REVERSE
        self.start_time = self.get_clock().now()
        self.start_x = self.dock_rel[0] if self.dock_rel else 0.0
        self._report('开始离桩倒车')
        response.success = True
        response.message = 'undock started'
        return response

    # ---------- 控制 ----------
    def _on_control(self) -> None:
        now = self.get_clock().now()
        cmd = Twist()

        if self.state == self.ST_IDLE:
            return

        # 位姿数据失效保护
        pose_age = (now - self.dock_rel_stamp).nanoseconds * 1e-9
        if self.state in (self.ST_ALIGN, self.ST_APPROACH, self.ST_FINAL, self.ST_REVERSE):
            if self.dock_rel is None or pose_age > self.get_parameter('pose_timeout_s').value:
                self._finish(False, '充电桩位姿数据失效')
                return

        x, y, dyaw = self.dock_rel

        if self.state == self.ST_ALIGN:
            target_yaw = math.atan2(y, x)
            if abs(target_yaw) < self.get_parameter('align_tolerance').value:
                self.state = self.ST_APPROACH
                self._report('朝向对齐完成, 进入接近段')
            else:
                cmd.angular.z = clamp(self.get_parameter('kp_yaw').value * target_yaw,
                                      -self.get_parameter('max_angular_vel').value,
                                      self.get_parameter('max_angular_vel').value)

        elif self.state in (self.ST_APPROACH, self.ST_FINAL):
            timeout = self.get_parameter('timeout_s').value
            if (now - self.start_time).nanoseconds * 1e-9 > timeout:
                self._finish(False, f'泊靠超时 ({timeout:.0f}s)')
                return
            # 前向碰撞急停
            if self.front_min_range < self.get_parameter('collision_stop_distance').value:
                self._finish(False, f'前向障碍过近 ({self.front_min_range:.2f}m), 急停')
                return

            xy_tol = self.get_parameter('xy_tolerance').value
            yaw_tol = self.get_parameter('yaw_tolerance').value
            if abs(x) <= xy_tol and abs(y) <= xy_tol and abs(dyaw) <= yaw_tol:
                self._finish(True, '泊靠成功')
                return

            # 曲率式同时修正：前向推进 + 横向偏差/朝向误差加权转向。
            # 不能用"对准目标方位角"的 bang-bang 方式：接近段目标方位角对
            # 横向小偏差过于敏感（kp_yaw 大、限幅饱和），会在目标附近极限环
            # 振荡，永远达不到 (x, y, yaw) 联合容差。
            cap = (self.get_parameter('final_linear_cap').value
                   if self.state == self.ST_FINAL else self.get_parameter('max_linear_vel').value)
            v = clamp(self.get_parameter('kp_dist').value * x, 0.0, cap)
            w = clamp(self.get_parameter('kp_yaw').value * dyaw
                      + self.get_parameter('kp_y').value * y,
                      -self.get_parameter('max_angular_vel').value,
                      self.get_parameter('max_angular_vel').value)
            # 朝向误差过大时先原地修正再推进，防止斜向蹭桩
            if abs(dyaw) > 1.0:
                v = 0.0
            cmd.linear.x = v
            cmd.angular.z = w
            if self.state == self.ST_APPROACH and math.hypot(x, y) < self.get_parameter('final_range').value:
                self.state = self.ST_FINAL
                self._report('进入末段精调')

        elif self.state == self.ST_REVERSE:
            timeout = self.get_parameter('undock_timeout_s').value
            if (now - self.start_time).nanoseconds * 1e-9 > timeout:
                self._finish(False, '离桩超时')
                return
            if abs(self.dock_rel[0]) > self.get_parameter('undock_distance').value:
                self._finish(True, '离桩完成')
                return
            cmd.linear.x = -0.12
            cmd.angular.z = clamp(0.8 * self.dock_rel[2],
                                  -self.get_parameter('max_angular_vel').value,
                                  self.get_parameter('max_angular_vel').value)

        self.cmd_pub.publish(cmd)

    def _finish(self, success: bool, message: str) -> None:
        self.cmd_pub.publish(Twist())   # 停止
        self.state = self.ST_DONE if success else self.ST_FAILED
        self.success_pub.publish(Bool(data=success))
        self._report(('成功: ' if success else '失败: ') + message)
        (self.get_logger().info if success else self.get_logger().error)(message)

    def _report(self, text: str) -> None:
        msg = String()
        msg.data = f'[{self.get_clock().now().nanoseconds * 1e-9:.1f}s] {self.state}: {text}'
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
