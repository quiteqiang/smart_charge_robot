#!/usr/bin/env python3
"""充电任务状态机节点（rclpy 薄壳）。

决策核心在 mission_logic.MissionStateMachine（纯 Python，不依赖 rclpy）：
壳只负责 ROS IO——订阅/服务/定时器/action client——把外部事件翻译成核心的
调用，再把核心返回的 Effect 列表翻译回 ROS 副作用（导航/服务调用/话题发布）。
状态机全部转换边可用 pytest 秒级覆盖（test/test_mission_logic.py），不必跑
端到端集成测试。见 docs/improvement_directions.md #4。

ROS 接口（与壳前实现一致）：
- 订阅：/battery_state、/docking_success（TRANSIENT_LOCAL + 世代校验）、
  /docking_status、/mission/goto
- 发布：/charging_active、/mission_state（TRANSIENT_LOCAL latched）、/status_text
- 服务：/mission/start_task、/mission/reset
- action：navigate_to_pose；TF 看门狗：map->base_link
"""
from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from smart_charge_msgs.msg import DockResult
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

import yaml

from smart_charge_mission.mission_logic import (
    ERROR_WAITING_HUMAN, IDLE, MissionStateMachine,
)

# 可运行时调参（ros2 param set）的配置项：参数名 -> (校验类型, 机器属性名)
_LIVE_PARAMS = {
    'low_soc_threshold': (Parameter.Type.DOUBLE, 'low_soc_threshold'),
    'resume_soc_threshold': (Parameter.Type.DOUBLE, 'resume_soc_threshold'),
    'max_docking_retries': (Parameter.Type.INTEGER, 'max_docking_retries'),
    'max_nav_retries': (Parameter.Type.INTEGER, 'max_nav_retries'),
    'dock_success_timeout_s': (Parameter.Type.DOUBLE, 'dock_success_timeout_s'),
    'charge_start_timeout_s': (Parameter.Type.DOUBLE, 'charge_start_timeout_s'),
    'task_waypoints': (Parameter.Type.STRING_ARRAY, 'task_waypoints'),
    'pre_dock_waypoint': (Parameter.Type.STRING, 'pre_dock_waypoint'),
}

# oneshot 定时器到点后分发的事件：now 由壳注入
_ONESHOT_EVENTS = {
    'begin_docking': lambda m, now, payload: m.begin_docking(now),
    'plan_charge_route': lambda m, now, payload: m.plan_charge_route(*payload),
    'retry_nav': lambda m, now, payload: m.retry_nav(*payload),
    'poll_dock': lambda m, now, payload: m.poll_dock(now),
    'poll_undock': lambda m, now, payload: m.poll_undock(now),
}


class ChargeMission(Node):
    def __init__(self) -> None:
        super().__init__('charge_mission')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('task_waypoints', ['work_1', 'work_2'])
        self.declare_parameter('pre_dock_waypoint', 'pre_dock')
        self.declare_parameter('low_soc_threshold', 0.25)
        self.declare_parameter('resume_soc_threshold', 0.85)
        self.declare_parameter('max_docking_retries', 3)
        self.declare_parameter('max_nav_retries', 2)
        self.declare_parameter('dock_success_timeout_s', 60.0)
        self.declare_parameter('charge_start_timeout_s', 15.0)
        self.declare_parameter('tf_loss_timeout_s', 5.0)
        self.declare_parameter('goal_reached_tolerance_m', 0.5)

        with open(self.get_parameter('waypoints_file').value, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.frame = data.get('frame', 'map')
        self.waypoints: dict[str, dict] = data['waypoints']

        def _log(level: str, msg: str) -> None:
            # rclpy 按调用点 (文件/行) 缓存 severity：同一行不得用不同级别
            # 记录（否则 ValueError: Logger severity cannot be changed between
            # calls），因此每个级别必须固定从各自的源码行发出
            logger = self.get_logger()
            if level == 'info':
                logger.info(msg)
            elif level == 'warn':
                logger.warn(msg)
            elif level == 'error':
                logger.error(msg)
            else:
                logger.debug(msg)

        self.machine = MissionStateMachine(
            waypoints=self.waypoints,
            task_waypoints=list(self.get_parameter('task_waypoints').value),
            pre_dock_waypoint=self.get_parameter('pre_dock_waypoint').value,
            low_soc_threshold=self.get_parameter('low_soc_threshold').value,
            resume_soc_threshold=self.get_parameter('resume_soc_threshold').value,
            max_docking_retries=self.get_parameter('max_docking_retries').value,
            max_nav_retries=self.get_parameter('max_nav_retries').value,
            dock_success_timeout_s=self.get_parameter('dock_success_timeout_s').value,
            charge_start_timeout_s=self.get_parameter('charge_start_timeout_s').value,
            tf_loss_timeout_s=self.get_parameter('tf_loss_timeout_s').value,
            log=_log,
        )
        self._last_tf_ok_time: float | None = None
        self._nav_goal_handle = None

        group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.dock_start = self.create_client(Trigger, '/dock/start')
        self.dock_undock = self.create_client(Trigger, '/dock/undock')

        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        # TRANSIENT_LOCAL 与发布端匹配：重订阅时能拿到 latched 基线/旧结果，
        # 由核心的世代校验负责丢弃过期值（volatile 订阅收不到历史，基线约定失效）
        self.create_subscription(
            DockResult, '/docking_success', self._on_dock_result,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/docking_status', self._on_docking_status, 10)
        self.create_subscription(String, '/mission/goto', self._on_goto, 10)

        self.charging_pub = self.create_publisher(Bool, '/charging_active', 10)
        # TRANSIENT_LOCAL：迟到订阅者（测试/RViz/battery 节点）能拿到当前状态
        self.state_pub = self.create_publisher(
            String, '/mission_state',
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.text_pub = self.create_publisher(MarkerArray, '/status_text', 10)

        self.create_service(Trigger, '/mission/start_task', self._on_start_task)
        self.create_service(Trigger, '/mission/reset', self._on_reset)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.5, self._watchdog)
        self.create_timer(1.0, self._publish_status_text)
        # 参数在壳前实现中是逐次 get_parameter 实时读取的；纯化后由
        # 此回调把运行时改动推进状态机，保持 ros2 param set 即时生效
        self.add_on_set_parameters_callback(self._on_set_params)
        # 发布初始状态，迟到订阅者可立即获得
        self.state_pub.publish(String(data=self.machine.state))
        self.get_logger().info('充电任务状态机就绪 (IDLE)，等待 /mission/start_task')

    def _on_set_params(self, params) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        result.reason = ''
        overrides: dict = {}
        for p in params:
            if p.type_ == Parameter.Type.NOT_SET or p.name not in _LIVE_PARAMS:
                continue   # 未识别参数走默认存储（与 declare 行为一致）
            expected, attr = _LIVE_PARAMS[p.name]
            if p.type_ != expected:
                result.successful = False
                result.reason = f'{p.name} 类型不符，期望 {expected.name}'
                continue
            overrides[attr] = list(p.value) if p.type_ == Parameter.Type.STRING_ARRAY else p.value
        if result.successful and overrides:
            self.machine.update_config(**overrides)
        return result

    # ------------------------------------------------------------ Effect 执行
    def _run(self, effects) -> None:
        """依次执行核心返回的副作用。"""
        for effect in effects:
            getattr(self, '_fx_' + effect.kind)(*effect.args)

    def _fx_state(self, new: str, reason: str) -> None:
        self.state_pub.publish(String(data=new))

    def _fx_nav_goal(self, name: str, source: str) -> None:
        self._send_nav_goal(name, source)

    def _fx_dock_start(self) -> None:
        if not self.dock_start.wait_for_service(timeout_sec=5.0):
            self._run(self.machine.dock_start_response(False, '泊靠服务不可用'))
            return
        future = self.dock_start.call_async(Trigger.Request())
        future.add_done_callback(self._on_dock_start_response)

    def _fx_undock(self) -> None:
        if not self.dock_undock.wait_for_service(timeout_sec=5.0):
            self._run(self.machine.undock_response(False, '离桩服务不可用'))
            return
        future = self.dock_undock.call_async(Trigger.Request())
        future.add_done_callback(self._on_undock_response)

    def _fx_charging(self, active: bool) -> None:
        self.charging_pub.publish(Bool(data=active))

    def _fx_oneshot(self, delay_s: float, event: str, payload: tuple) -> None:
        def wrapper():
            self.destroy_timer(timer)
            now = self.get_clock().now().nanoseconds * 1e-9
            self._run(_ONESHOT_EVENTS[event](self.machine, now, payload))
        timer = self.create_timer(delay_s, wrapper)

    def _fx_cancel_nav(self) -> None:
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    # ------------------------------------------------------------ 导航 IO
    def _send_nav_goal(self, name: str, source: str) -> bool:
        pose = self._wp_pose(name)
        if pose is None:
            return False   # 未知航点：原行为仅报错，状态不变（配置期保证合法性）
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action 不可用')
            self._run(self.machine.nav_unavailable(name, source))
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_goal_handle = None
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_nav_goal_response(f, name, source))
        self.get_logger().info(
            f'[{source}] 导航至 {name} ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})')
        self.machine.nav_sent(name, source)
        return True

    def _on_nav_goal_response(self, future, name: str, source: str) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'导航目标 {name} 被拒绝')
            self._nav_goal_handle = None
            self._run(self.machine.nav_done(False, name, source))
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_nav_result(f, name, source, handle))

    def _on_nav_result(self, future, name: str, source: str, handle) -> None:
        # 句柄身份比较：goto/start_task 抢占后，被取消旧目标的迟到结果
        # 不得清掉新目标的句柄——否则错误态/下次抢占的 cancel 会落空，
        # Nav2 继续执行已过期目标（review finding 2，#7）
        if self._nav_goal_handle is handle:
            self._nav_goal_handle = None
        try:
            result = future.result()
            ok = result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception as exc:  # noqa: BLE001 - action 结果异常统一按失败处理
            self.get_logger().error(f'导航结果异常: {exc}')
            ok = False
        self._run(self.machine.nav_done(ok, name, source))

    def _wp_pose(self, name: str) -> PoseStamped | None:
        wp = self.waypoints.get(name)
        if wp is None:
            self.get_logger().error(f'未知航点: {name}')
            return None
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.orientation = self._yaw_to_quat(float(wp['yaw']))
        return pose

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ------------------------------------------------------------ 服务/话题回调
    def _on_start_task(self, _, response: Trigger.Response) -> Trigger.Response:
        ok, message, effects = self.machine.start_task()
        self._run(effects)
        response.success = ok
        response.message = message
        return response

    def _on_reset(self, _, response: Trigger.Response) -> Trigger.Response:
        ok, message, effects = self.machine.reset()
        self._run(effects)
        response.success = ok
        response.message = message
        return response

    def _on_goto(self, msg: String) -> None:
        self._run(self.machine.goto(msg.data.strip()))

    def _on_battery(self, msg: BatteryState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self._run(self.machine.battery(msg.percentage, charging, now))

    def _on_dock_result(self, msg: DockResult) -> None:
        self._run(self.machine.dock_result(msg.sequence, msg.success))

    def _on_docking_status(self, msg: String) -> None:
        self.get_logger().info(f'[泊靠] {msg.data}')

    def _on_dock_start_response(self, future) -> None:
        try:
            ok, reason = future.result().success, ''
        except Exception as exc:  # noqa: BLE001
            ok, reason = False, f'泊靠服务调用异常: {exc}'
        if not ok and not reason:
            reason = '泊靠服务拒绝启动'
        self._run(self.machine.dock_start_response(ok, reason))

    def _on_undock_response(self, future) -> None:
        try:
            ok, reason = future.result().success, ''
        except Exception as exc:  # noqa: BLE001
            ok, reason = False, f'离桩调用异常: {exc}'
        if not ok and not reason:
            reason = '离桩被拒绝'
        self._run(self.machine.undock_response(ok, reason))

    # ------------------------------------------------------------ 看门狗
    def _watchdog(self) -> None:
        # 定位看门狗：map->base_link TF 丢失视为定位失效
        if self.machine.state in (IDLE, ERROR_WAITING_HUMAN):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                            timeout=Duration(seconds=0.2))
            self._last_tf_ok_time = now
        except TransformException:
            pass
        if self._last_tf_ok_time is None:
            return
        lost = now - self._last_tf_ok_time
        if lost > self.get_parameter('tf_loss_timeout_s').value:
            self._run(self.machine.tf_lost(lost))

    # ------------------------------------------------------------ RViz 文本
    def _publish_status_text(self) -> None:
        arr = MarkerArray()
        for i, text in enumerate((f'SOC: {self.machine.soc:.0%}',
                                  f'State: {self.machine.state}')):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.ns = 'mission_status'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.z = 1.0 + 0.35 * i
            m.scale.z = 0.3
            m.color = ColorRGBA(a=1.0, r=1.0 if i == 0 else 0.2, g=0.9, b=0.2)
            m.text = text
            m.lifetime.sec = 2
            arr.markers.append(m)
        self.text_pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = ChargeMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
