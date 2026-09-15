#!/usr/bin/env python3
"""电池 SOC 仿真节点。

行为模型：
  - 行驶放电：SOC 以 `discharge_rate * (|v| / max_linear_speed)` 的速率下降；
  - 待机放电：静止时以 `idle_discharge_rate` 缓慢下降；
  - 充电：仅当 /dock_contact 为 True（已成功泊靠充电桩）时，以 `charge_rate` 上升；
  - 所有速率参数可调；SOC 通过 /set_soc 服务可在运行时注入（测试必备）。

发布：
  - /battery_state (sensor_msgs/BatteryState)：percentage、电压、充放电状态；
  - /battery_text_markers (visualization_msgs/Marker)：RViz 文本显示（SOC% + 状态）。

真实硬件替换方式：停用本节点，将真实 BMS 驱动发布到相同话题即可，
任务状态机只依赖 /battery_state 与 /dock_contact 两个接口。
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from smart_charge_msgs.srv import SetSoc
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker


class BatterySimulator(Node):

    def __init__(self) -> None:
        super().__init__('battery_simulator')

        self.declare_parameter('initial_soc', 1.0)
        self.declare_parameter('discharge_rate', 0.002)
        self.declare_parameter('idle_discharge_rate', 0.0001)
        self.declare_parameter('charge_rate', 0.01)
        self.declare_parameter('low_soc_threshold', 0.25)
        self.declare_parameter('resume_soc_threshold', 0.85)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('publish_rate', 2.0)

        self._soc = float(self.get_parameter('initial_soc').value)
        self._discharge_rate = float(self.get_parameter('discharge_rate').value)
        self._idle_discharge_rate = float(self.get_parameter('idle_discharge_rate').value)
        self._charge_rate = float(self.get_parameter('charge_rate').value)
        self._max_speed = float(self.get_parameter('max_linear_speed').value)
        self._low_thr = float(self.get_parameter('low_soc_threshold').value)
        self._resume_thr = float(self.get_parameter('resume_soc_threshold').value)

        self._speed = 0.0
        self._docked = False          # /dock_contact：充电枪物理接触
        self._charge_requested = False  # /charging_active：任务机请求开始充电
        self._prev_time = self.get_clock().now()

        self.create_subscription(Bool, '/dock_contact', self._on_dock_contact, 10)
        self.create_subscription(Bool, '/charging_active', self._on_charging_active, 10)
        # /cmd_vel 订阅用于速度比例放电；兼容 TwistStamped（Nav2 平滑输出）
        from geometry_msgs.msg import Twist, TwistStamped
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_smoothed',
                                 self._on_cmd_vel_stamped, 10)

        self._battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self._text_pub = self.create_publisher(Marker, '/battery_text_markers', 10)
        self._status_pub = self.create_publisher(String, '/battery_status_text', 10)
        self.create_service(SetSoc, '/set_soc', self._on_set_soc)

        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                          self._tick)
        self.get_logger().info(
            f'电池仿真启动: SOC={self._soc:.2f}, 低电量阈值={self._low_thr:.2f}, '
            f'恢复阈值={self._resume_thr:.2f}')

    # ------------------------------------------------------------------ #
    def _on_cmd_vel(self, msg) -> None:
        self._speed = math.hypot(msg.linear.x, msg.linear.y)

    def _on_cmd_vel_stamped(self, msg) -> None:
        self._speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def _on_dock_contact(self, msg: Bool) -> None:
        if msg.data != self._docked:
            self.get_logger().info(
                f'充电接触状态: {self._docked} -> {msg.data}')
        self._docked = msg.data

    def _on_charging_active(self, msg: Bool) -> None:
        if msg.data != self._charge_requested:
            self.get_logger().info(
                f'充电请求: {self._charge_requested} -> {msg.data}')
        self._charge_requested = msg.data

    def _on_set_soc(self, req: SetSoc.Request,
                    res: SetSoc.Response) -> SetSoc.Response:
        if not (0.0 <= req.soc <= 1.0):
            res.success = False
            res.message = f'SOC {req.soc} 超出 [0,1] 范围'
            return res
        self._soc = req.soc
        res.success = True
        res.message = f'SOC 已设置为 {self._soc:.2f}'
        self.get_logger().warn(f'[测试注入] SOC = {self._soc:.2f}')
        return res

    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        dt = min(dt, 1.0)  # 防止暂停后续电跳变

        # 速率参数每次实时读取：测试可在运行时调参加速充放电
        charge_rate = float(self.get_parameter('charge_rate').value)
        discharge_rate = float(self.get_parameter('discharge_rate').value)
        idle_rate = float(self.get_parameter('idle_discharge_rate').value)

        charging = self._docked and self._charge_requested and self._soc < 1.0
        if self._docked and not self._charge_requested and self._soc < 1.0:
            # 已连接但任务机未拉起充电请求：保持当前电量，等待握手
            pass
        elif charging:
            self._soc = min(1.0, self._soc + charge_rate * dt)
        else:
            scale = min(self._speed / self._max_speed, 1.0) if self._max_speed > 0 else 0.0
            rate = idle_rate + (discharge_rate - idle_rate) * scale
            self._soc = max(0.0, self._soc - rate * dt)

        self._publish(now)

    def _publish(self, now) -> None:
        charging = self._docked and self._charge_requested and self._soc < 1.0
        status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging else \
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = 22.0 + 4.0 * self._soc          # 22V(空) ~ 26V(满)，示例
        msg.percentage = float(self._soc)
        msg.capacity = 100.0
        msg.design_capacity = 100.0
        msg.power_supply_status = status
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self._battery_pub.publish(msg)

        text = f'SOC: {self._soc * 100.0:5.1f}%  ' + ('⚡CHARGING' if charging else 'DISCHARGING')
        if self._soc < self._low_thr:
            text += '  [LOW!]'
        self._status_pub.publish(String(data=text))

        m = Marker()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'base_link'
        m.ns = 'battery'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 1.2
        m.pose.orientation.w = 1.0
        m.scale.z = 0.4
        m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.9, 0.2, 1.0) if not charging \
            else (0.1, 0.5, 1.0, 1.0)
        m.text = text
        self._text_pub.publish(m)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
