#!/usr/bin/env python3
"""电池 SOC 仿真节点（rclpy 薄壳）。

电池行为模型在 battery_model.BatteryModel（纯 Python，不依赖 rclpy）：
CC/CV 两段充电曲线、电压一阶滞后 + 负载压降、Ah 电荷记账——见
docs/improvement_directions.md #6。壳只负责 ROS IO：
  - 订阅 /cmd_vel、/cmd_vel_smoothed：速度比例放电；
  - 订阅 /dock_contact：充电枪物理接触；
  - 订阅 /charging_active：任务机请求开始充电；
  - 订阅 /mission_state（TRANSIENT_LOCAL）：仅用于 [LOW!] 显示推导；
  - 服务 /set_soc：运行时注入 SOC（测试必备）；
  - 发布 /battery_state (sensor_msgs/BatteryState)、/battery_text_markers、
    /battery_status_text。

阈值单一事实源：低电量/恢复阈值只由 charge_mission 节点持有并决策，
本节点不持有决策副本；[LOW!] 提示改为订阅 /mission_state 推导
（低电量充电循环进行中 = LOW_BATTERY/NAVIGATING_TO_DOCK/PRE_DOCKING/DOCKING），
避免"电池节点与状态机阈值不一致"的维护陷阱（improvement_directions #3）。

降级兜底：mission 节点未运行（''）或已进入人工错误态时，充电循环状态推导
失效，此时用独立的 display-only 阈值 low_soc_display_threshold（刻意低于决策
阈值，仅作"电量即将耗尽"的醒目提示，不参与任何决策）恢复提示能力。

真实硬件替换方式：停用本节点，将真实 BMS 驱动发布到相同话题即可，
任务状态机只依赖 /battery_state 与 /dock_contact 两个接口。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import BatteryState
from smart_charge_msgs.srv import SetSoc
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from smart_charge_battery.battery_model import BatteryModel

# 低电量充电循环中的 mission 状态（此时 SOC 必然低于 low_soc_threshold，
# 阈值由 mission 节点单一持有，本节点只做显示推导）
_CHARGE_CYCLE_STATES = frozenset({
    'LOW_BATTERY', 'NAVIGATING_TO_DOCK', 'PRE_DOCKING', 'DOCKING',
})

# mission 状态推导失效（节点离线/人工错误态）时的降级显示阈值：
# display-only，刻意低于决策阈值 0.25，不参与任何决策
_DEGRADED_STATES = frozenset({'', 'ERROR_WAITING_HUMAN'})


class BatterySimulator(Node):

    def __init__(self) -> None:
        super().__init__('battery_simulator')

        self.declare_parameter('initial_soc', 1.0)
        self.declare_parameter('capacity_ah', 100.0)
        self.declare_parameter('discharge_rate', 0.002)
        self.declare_parameter('idle_discharge_rate', 0.0001)
        self.declare_parameter('charge_rate', 0.01)
        self.declare_parameter('cc_cv_threshold', 0.8)
        self.declare_parameter('internal_resistance', 2.0)
        self.declare_parameter('voltage_tau_s', 2.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('low_soc_display_threshold', 0.10)

        self._model = BatteryModel(
            capacity_ah=float(self.get_parameter('capacity_ah').value),
            initial_soc=float(self.get_parameter('initial_soc').value),
            max_speed=float(self.get_parameter('max_linear_speed').value),
            cc_cv_threshold=float(self.get_parameter('cc_cv_threshold').value),
            internal_resistance=float(self.get_parameter('internal_resistance').value),
            voltage_tau_s=float(self.get_parameter('voltage_tau_s').value),
        )
        self._low_display_thr = float(
            self.get_parameter('low_soc_display_threshold').value)

        self._speed = 0.0
        self._docked = False          # /dock_contact：充电枪物理接触
        self._charge_requested = False  # /charging_active：任务机请求开始充电
        self._mission_state = ''      # /mission_state：仅用于 [LOW!] 显示推导
        self._prev_time = self.get_clock().now()

        self.create_subscription(Bool, '/dock_contact', self._on_dock_contact, 10)
        self.create_subscription(Bool, '/charging_active', self._on_charging_active, 10)
        # TRANSIENT_LOCAL 与 mission 端匹配：迟到订阅也能拿到当前状态
        self.create_subscription(
            String, '/mission_state', self._on_mission_state,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
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
            f'电池仿真启动: SOC={self._model.soc:.2f} '
            f'(低电量阈值由 charge_mission 单一持有，本节点仅做显示推导)')

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

    def _on_mission_state(self, msg: String) -> None:
        self._mission_state = msg.data

    def _on_set_soc(self, req: SetSoc.Request,
                    res: SetSoc.Response) -> SetSoc.Response:
        if not (0.0 <= req.soc <= 1.0):
            res.success = False
            res.message = f'SOC {req.soc} 超出 [0,1] 范围'
            return res
        self._model.set_soc(req.soc)
        res.success = True
        res.message = f'SOC 已设置为 {self._model.soc:.2f}'
        self.get_logger().warn(f'[测试注入] SOC = {self._model.soc:.2f}')
        return res

    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        dt = min(dt, 1.0)  # 防止暂停后续电跳变

        # 速率参数每次实时读取：测试可在运行时调参加速充放电
        snap = self._model.step(
            dt, speed=self._speed, docked=self._docked,
            charge_requested=self._charge_requested,
            charge_rate=float(self.get_parameter('charge_rate').value),
            discharge_rate=float(self.get_parameter('discharge_rate').value),
            idle_discharge_rate=float(
                self.get_parameter('idle_discharge_rate').value))
        self._publish(now, snap)

    def _publish(self, now, snap) -> None:
        status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if snap.charging else \
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = snap.voltage
        msg.current = snap.current          # 正 = 放电，负 = 充电 (A)
        msg.percentage = float(snap.soc)
        msg.capacity = float(self._model.capacity_ah)
        msg.design_capacity = float(self._model.capacity_ah)
        msg.power_supply_status = status
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self._battery_pub.publish(msg)

        text = (f'SOC: {snap.soc * 100.0:5.1f}%  '
                + ('⚡CHARGING' if snap.charging else 'DISCHARGING'))
        if (self._mission_state in _CHARGE_CYCLE_STATES
                or (self._mission_state in _DEGRADED_STATES
                    and snap.soc < self._low_display_thr)):
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
        m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.9, 0.2, 1.0) \
            if not snap.charging else (0.1, 0.5, 1.0, 1.0)
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
