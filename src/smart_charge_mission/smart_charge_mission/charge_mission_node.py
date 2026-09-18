#!/usr/bin/env python3
"""充电任务状态机。

状态（与 docs/architecture.md 状态图一致）：
  IDLE → EXECUTING_TASK ⇄ (低电量) → NAVIGATING_TO_DOCK → PRE_DOCKING → DOCKING
       → CHARGING → UNDOCKING → RESUMING_TASK → EXECUTING_TASK
  任意导航/泊靠失败重试耗尽 → ERROR_WAITING_HUMAN（等待人工 /mission/reset）

关键设计：
- 导航走 Nav2 NavigateToPose action；低电量时 cancel 当前 goal 并保存航点；
- 泊靠走 dock_controller 服务，最多重试 max_docking_retries 次；
- "开始充电确认" = battery 节点回报 POWER_SUPPLY_STATUS_CHARGING（本节点先拉起
  /charging_active 请求充电，确认在充电状态进入前完成握手）；
- 泊靠/离桩结果带单调 sequence（/docking_success, DockResult）：只接受晚于
  本次请求发出时刻的结果，丢弃重订阅时 replay 的 latched 旧值，避免假成功；
- 所有转换打结构化日志，RViz 通过 /status_text 文本标记展示 SOC 与状态。
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
from sensor_msgs.msg import BatteryState
from smart_charge_msgs.msg import DockResult
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

import yaml

# 状态常量
IDLE = 'IDLE'
EXECUTING_TASK = 'EXECUTING_TASK'
LOW_BATTERY = 'LOW_BATTERY'
NAVIGATING_TO_DOCK = 'NAVIGATING_TO_DOCK'
PRE_DOCKING = 'PRE_DOCKING'
DOCKING = 'DOCKING'
CHARGING = 'CHARGING'
UNDOCKING = 'UNDOCKING'
RESUMING_TASK = 'RESUMING_TASK'
ERROR_WAITING_HUMAN = 'ERROR_WAITING_HUMAN'


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

        self.state = IDLE
        self.soc = 1.0
        self.battery_ok = False
        self.task_queue: list[str] = []
        self.saved_task: str | None = None
        self._low_battery_dwell_id = 0   # LOW_BATTERY 停留世代（孤儿 oneshot 防护）
        self.nav_retries = 0
        self.dock_retries = 0
        # 泊靠/离桩结果：(epoch, sequence, success)；配合请求基线做世代校验：
        # - sequence 单调递增，丢弃重订阅 replay 的 latched 旧值（防假成功）；
        # - epoch 在序号回退（控制器重启、序号空间重置）时递增，否则重启后
        #   所有真实结果都会因序号 "落后于旧基线" 被永久拒绝
        self.pending_dock_result: tuple[int, int, bool] | None = None
        self.pending_undock_result: tuple[int, int, bool] | None = None
        self._result_epoch = 0         # 结果世代（控制器重启检测）
        self._last_result_seq = 0      # 当前世代内最近收到的序号
        self._dock_req_epoch = 0       # 本次泊靠请求发出时的基线
        self._dock_req_seq = 0
        self._undock_req_epoch = 0     # 本次离桩请求发出时的基线
        self._undock_req_seq = 0
        self._nav_goal_handle = None
        self._active_nav_target: str | None = None
        self._dock_wait_start = None
        self._undock_wait_start = None
        self._last_tf_ok_time = None
        self._charge_request_time = None

        group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.dock_start = self.create_client(Trigger, '/dock/start')
        self.dock_undock = self.create_client(Trigger, '/dock/undock')

        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        # TRANSIENT_LOCAL 与发布端匹配：重订阅时能拿到 latched 基线/旧结果，
        # 由下面的世代校验负责丢弃过期值（volatile 订阅收不到历史，基线约定失效）
        self.create_subscription(
            DockResult, '/docking_success', self._on_dock_result,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/docking_status', self._on_docking_status, 10)
        self.create_subscription(String, '/mission/goto', self._on_goto, 10)

        self.charging_pub = self.create_publisher(Bool, '/charging_active', 10)
        # TRANSIENT_LOCAL：迟到订阅者（测试/RViz）能拿到当前状态
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
        # 发布初始状态，迟到订阅者可立即获得
        self.state_pub.publish(String(data=self.state))
        self.get_logger().info('充电任务状态机就绪 (IDLE)，等待 /mission/start_task')


    # ------------------------------------------------------------ 工具
    def _oneshot(self, delay_s: float, fn) -> None:
        """rclpy 无 oneshot 定时器：触发后自毁。"""
        def wrapper():
            self.destroy_timer(timer)
            fn()
        timer = self.create_timer(delay_s, wrapper)

    def _set_state(self, new: str, reason: str = '') -> None:
        if new == self.state:
            return
        self.get_logger().warn(f'状态转换: {self.state} -> {new}  {reason}')
        self.state = new
        msg = String()
        msg.data = new
        self.state_pub.publish(msg)

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

    def _send_nav_goal(self, name: str, source: str) -> bool:
        pose = self._wp_pose(name)
        if pose is None:
            return False
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action 不可用')
            self._enter_error('Nav2 action 不可用')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_goal_handle = None
        self._active_nav_target = name
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_nav_goal_response(f, name, source))
        self.get_logger().info(f'[{source}] 导航至 {name} ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})')
        return True

    def _on_nav_goal_response(self, future, name: str, source: str) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'导航目标 {name} 被拒绝')
            self._on_nav_done(False, name, source)
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result(f, name, source))

    def _on_nav_result(self, future, name: str, source: str) -> None:
        try:
            result = future.result()
            ok = result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception as exc:  # noqa: BLE001 - action 结果异常统一按失败处理
            self.get_logger().error(f'导航结果异常: {exc}')
            ok = False
        self._on_nav_done(ok, name, source)

    def _on_nav_done(self, success: bool, name: str, source: str) -> None:
        self._nav_goal_handle = None
        self._active_nav_target = None
        self.get_logger().warn(f'导航 {name} [{"成功" if success else "失败/取消"}] (来源 {source})')
        # 结果与当前状态不匹配（如低电量已取消并转充电）则忽略，防止误重试
        if source == 'task' and self.state not in (EXECUTING_TASK, RESUMING_TASK):
            return
        if source == 'dock' and self.state != NAVIGATING_TO_DOCK:
            return
        if success:
            self.nav_retries = 0
            if source == 'task' and self.state in (EXECUTING_TASK, RESUMING_TASK):
                if self.saved_task is not None:
                    self.get_logger().info(f'已恢复被暂停任务: {self.saved_task}')
                    self.saved_task = None
                self._advance_task()
            elif source == 'dock' and self.state == NAVIGATING_TO_DOCK:
                self._set_state(PRE_DOCKING, '到达充电桩预停靠点')
                self._oneshot(0.5, self._begin_docking)
            elif source == 'resume_none' and self.state == RESUMING_TASK:
                self._set_state(IDLE, '无被暂停任务，回到空闲')
            return
        # 失败重试
        self.nav_retries += 1
        if self.nav_retries >= self.get_parameter('max_nav_retries').value:
            self._enter_error(f'导航 {name} 连续失败 {self.nav_retries} 次')
        else:
            self.get_logger().warn(f'导航重试 {self.nav_retries}/{self.get_parameter("max_nav_retries").value}: {name}')
            self._oneshot(1.0, lambda: self._retry_nav_goal(name, source))

    def _retry_nav_goal(self, name: str, source: str) -> None:
        '''导航重试前守卫：1s 等待期间状态可能已变（如低电量转充电），
        旧目标的迟到重试不能再发出新导航。'''
        expected = {'task': (EXECUTING_TASK, RESUMING_TASK),
                    'dock': (NAVIGATING_TO_DOCK,),
                    'resume_none': (RESUMING_TASK,)}[source]
        if self.state not in expected:
            return
        self._send_nav_goal(name, source)

    def _advance_task(self) -> None:
        if not self.task_queue:
            self._set_state(IDLE, '全部任务完成')
            return
        name = self.task_queue.pop(0)
        self._set_state(EXECUTING_TASK, f'执行任务 {name}')
        self._send_nav_goal(name, 'task')

    # ------------------------------------------------------------ 服务/话题回调
    def _on_start_task(self, _, response: Trigger.Response) -> Trigger.Response:
        if self.state != IDLE:
            response.success = False
            response.message = f'当前状态 {self.state}，无法开始任务'
            return response
        self.task_queue = list(self.get_parameter('task_waypoints').value)
        self.get_logger().info(f'任务队列: {self.task_queue}')
        self._advance_task()
        response.success = True
        response.message = 'task started'
        return response

    def _on_reset(self, _, response: Trigger.Response) -> Trigger.Response:
        if self.state != ERROR_WAITING_HUMAN:
            response.success = False
            response.message = '仅错误态可复位'
            return response
        self.saved_task = None
        self.task_queue.clear()
        self.nav_retries = self.dock_retries = 0
        self._set_state(IDLE, '人工复位')
        response.success = True
        response.message = 'reset to IDLE'
        return response

    def _on_goto(self, msg: String) -> None:
        name = msg.data.strip()
        if name not in self.waypoints:
            self.get_logger().error(f'goto 未知航点: {name}')
            return
        if self.state not in (IDLE, EXECUTING_TASK):
            self.get_logger().warn(f'goto {name} 被忽略：状态 {self.state}')
            return
        if self.state == IDLE:
            self.task_queue = [name]
            self._advance_task()
        else:
            if name == self._active_nav_target:
                return  # 忽略重复目标（连发/双击），防止不必要的抢占
            self._send_nav_goal(name, 'task')

    def _on_battery(self, msg: BatteryState) -> None:
        if math.isnan(msg.percentage) or not 0.0 <= msg.percentage <= 1.0:
            self._enter_error(f'电量数据异常: {msg.percentage}')
            return
        self.soc = msg.percentage
        self.battery_ok = True
        low = self.get_parameter('low_soc_threshold').value
        resume = self.get_parameter('resume_soc_threshold').value

        if self.soc < low and self.state in (IDLE, EXECUTING_TASK):
            # 暂停当前任务：保存航点，当前导航目标由随后下发的充电桩目标抢占
            # 以 _active_nav_target 判定而非 _nav_goal_handle：目标已发出但尚未被接受时
            # handle 仍为 None（见 _send_nav_goal），此时航点同样需要保存
            if self.state == EXECUTING_TASK and self._active_nav_target is not None:
                self.saved_task = self._active_nav_target
                # 不显式 cancel：navigate_to_pose 是单目标服务端，下发充电桩目标即抢占并
                # 终结当前任务目标。若在此处 cancel_goal_async()，取消请求会晚 ~30 ms 到达，
                # 届时服务端的当前目标已是 pre_dock —— 取消会连带打掉刚发出的充电桩目标，
                # 白白消耗一次 max_nav_retries（见 issue #6）。被抢占的任务目标结果由
                # _on_nav_done 的 source/state 过滤丢弃，不会触发重试。
                self.get_logger().warn(f'低电量 {self.soc:.0%}，抢占当前导航并保存任务 {self.saved_task}')
            else:
                self.get_logger().warn(f'低电量 {self.soc:.0%}，无活动任务，直接前往充电')
            # 保留 task_queue：充电期间无任何路径消费它，
            # 恢复被暂停航点后 _on_nav_done 会调用 _advance_task 续跑剩余任务
            # LOW_BATTERY 是真实停留状态（"决策中"）：短暂停留后由
            # _plan_charge_route 起导航，与 docs/architecture.md 状态图一致；
            # 之前在同一个回调内被立即覆盖成 NAVIGATING_TO_DOCK，从不真实存在
            self._set_state(LOW_BATTERY, f'SOC={self.soc:.0%} < {low:.0%}')
            self._low_battery_dwell_id += 1
            dwell_id = self._low_battery_dwell_id
            self._oneshot(1.0, lambda: self._plan_charge_route(dwell_id))

        if self.state == CHARGING and self.soc >= resume:
            self.get_logger().info(f'SOC 达到恢复阈值 {self.soc:.0%} >= {resume:.0%}，结束充电')
            self._set_state(UNDOCKING, '充电完成，离桩')
            self._request_undock()

        if self.state == DOCKING and self._charge_request_time is not None:
            # 等待充电握手确认：电池回报 CHARGING
            if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                self._charge_request_time = None
                self._set_state(CHARGING, '充电桩确认开始充电')
            elif (self.get_clock().now() - self._charge_request_time).nanoseconds * 1e-9 > \
                    self.get_parameter('charge_start_timeout_s').value:
                self._charge_request_time = None
                self._enter_error('充电桩未确认开始充电（超时）')

    def _plan_charge_route(self, dwell_id: int) -> None:
        # 低电量决策停留结束。守卫状态 + 停留世代：期间可能已被看门狗转入错误态，
        # 或上一次停留的孤儿 oneshot 在新的停留内触发（无世代守卫会提前起导航）
        if self.state != LOW_BATTERY or dwell_id != self._low_battery_dwell_id:
            return
        self._set_state(NAVIGATING_TO_DOCK, '规划充电路径')
        self.nav_retries = 0
        self.dock_retries = 0
        if not self._send_nav_goal(self.get_parameter('pre_dock_waypoint').value, 'dock'):
            self._enter_error('无法规划充电路径：预停靠点无效或 Nav2 不可用')

    def _on_dock_result(self, msg: DockResult) -> None:
        # 泊靠与离桩互斥进行：两个 pending 都更新，轮询时按世代基线各取所需
        if msg.sequence < self._last_result_seq:
            # 序号回退 = 控制器重启、序号空间重置：旧基线失效，进入新世代。
            # （重启后 in-flight 的少量旧消息可能再触发一次，方向是 fail-safe：
            # 结果按"新"被接受而非被永久拒绝，且泊靠结果仍由控制器状态机背书）
            self._result_epoch += 1
        self._last_result_seq = msg.sequence
        self.pending_dock_result = (self._result_epoch, msg.sequence, msg.success)
        self.pending_undock_result = (self._result_epoch, msg.sequence, msg.success)

    def _on_docking_status(self, msg: String) -> None:
        self.get_logger().info(f'[泊靠] {msg.data}')

    @staticmethod
    def _result_newer(result: tuple[int, int, bool], req_epoch: int, req_seq: int) -> bool:
        '''结果是否晚于请求基线（跨世代或同世代序号更新）。'''
        return result[0] > req_epoch or (result[0] == req_epoch and result[1] > req_seq)

    # ------------------------------------------------------------ 泊靠
    def _begin_docking(self) -> None:
        if self.state != PRE_DOCKING:
            return
        self._set_state(DOCKING, f'第 {self.dock_retries + 1} 次泊靠尝试')
        self.pending_dock_result = None
        self._dock_req_epoch = self._result_epoch
        self._dock_req_seq = self._last_result_seq
        self._dock_wait_start = self.get_clock().now()
        if not self.dock_start.wait_for_service(timeout_sec=5.0):
            self._dock_failed('泊靠服务不可用')
            return
        future = self.dock_start.call_async(Trigger.Request())
        future.add_done_callback(self._on_dock_start_response)

    def _on_dock_start_response(self, future) -> None:
        try:
            ok = future.result().success
        except Exception as exc:  # noqa: BLE001
            self._dock_failed(f'泊靠服务调用异常: {exc}')
            return
        if not ok:
            self._dock_failed('泊靠服务拒绝启动')
            return
        self._oneshot(0.5, self._poll_dock_result)

    def _poll_dock_result(self) -> None:
        if self.state != DOCKING:
            return
        result = self.pending_dock_result
        # 世代校验：同世代且序号不晚于请求基线的是 latched 旧值（重订阅 replay），忽略
        if result is not None and self._result_newer(result, self._dock_req_epoch, self._dock_req_seq):
            if result[2]:
                self._dock_succeeded()
            else:
                self._dock_failed('泊靠控制器报告失败')
            return
        # 总超时保护（控制器自身有超时，这里兜底防死等）
        if (self.get_clock().now() - self._dock_wait_start).nanoseconds * 1e-9 > \
                self.get_parameter('dock_success_timeout_s').value:
            self._dock_wait_start = None
            self._dock_failed('等待泊靠结果超时')
            return
        self._oneshot(0.5, self._poll_dock_result)

    def _dock_succeeded(self) -> None:
        self.dock_retries = 0
        self.get_logger().info('泊靠成功，请求开始充电')
        self.charging_pub.publish(Bool(data=True))
        self._charge_request_time = self.get_clock().now()   # 等电池节点回报 CHARGING

    def _dock_failed(self, reason: str) -> None:
        self.charging_pub.publish(Bool(data=False))
        self.dock_retries += 1
        max_retries = self.get_parameter('max_docking_retries').value
        self.get_logger().error(f'泊靠失败({reason})，第 {self.dock_retries}/{max_retries} 次')
        if self.dock_retries >= max_retries:
            self._enter_error(f'泊靠重试 {max_retries} 次均失败: {reason}')
            return
        # 退回预停靠点重新尝试
        self._set_state(NAVIGATING_TO_DOCK, f'泊靠失败重试 {self.dock_retries}/{max_retries}')
        self._send_nav_goal(self.get_parameter('pre_dock_waypoint').value, 'dock')

    # ------------------------------------------------------------ 离桩/恢复
    def _request_undock(self) -> None:
        self.charging_pub.publish(Bool(data=False))
        self.pending_undock_result = None
        self._undock_req_epoch = self._result_epoch
        self._undock_req_seq = self._last_result_seq
        self._undock_wait_start = self.get_clock().now()
        if not self.dock_undock.wait_for_service(timeout_sec=5.0):
            self._enter_error('离桩服务不可用')
            return
        future = self.dock_undock.call_async(Trigger.Request())
        future.add_done_callback(self._on_undock_response)
        self._oneshot(0.5, self._poll_undock_result)

    def _on_undock_response(self, future) -> None:
        try:
            if not future.result().success:
                self._enter_error('离桩被拒绝')
        except Exception as exc:  # noqa: BLE001
            self._enter_error(f'离桩调用异常: {exc}')

    def _poll_undock_result(self) -> None:
        if self.state != UNDOCKING:
            return
        # 超时兜底：控制器可能在受理离桩后、发布结果前死亡，无超时则永久挂起
        if (self.get_clock().now() - self._undock_wait_start).nanoseconds * 1e-9 > \
                self.get_parameter('dock_success_timeout_s').value:
            self._undock_wait_start = None
            self._enter_error('等待离桩结果超时')
            return
        result = self.pending_undock_result
        # 世代校验：同 _poll_dock_result，忽略 latched 旧值
        if result is not None and self._result_newer(result, self._undock_req_epoch, self._undock_req_seq):
            if result[2]:
                self._resume_task()
            else:
                self._enter_error('离桩失败')
            return
        self._oneshot(0.5, self._poll_undock_result)

    def _resume_task(self) -> None:
        if self.saved_task is not None:
            self._set_state(RESUMING_TASK, f'返回被暂停任务 {self.saved_task}')
            self._send_nav_goal(self.saved_task, 'task')
        else:
            self._set_state(RESUMING_TASK, '无被暂停任务')
            self._send_nav_goal(self.get_parameter('pre_dock_waypoint').value, 'resume_none')

    # ------------------------------------------------------------ 看门狗
    def _watchdog(self) -> None:
        # 定位看门狗：map->base_link TF 丢失视为定位失效
        if self.state in (IDLE, ERROR_WAITING_HUMAN):
            return
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                            timeout=Duration(seconds=0.2))
            self._last_tf_ok_time = self.get_clock().now()
        except TransformException:
            pass
        if self._last_tf_ok_time is None:
            return
        lost = (self.get_clock().now() - self._last_tf_ok_time).nanoseconds * 1e-9
        if lost > self.get_parameter('tf_loss_timeout_s').value:
            self._enter_error(f'定位失效: map->base_link TF 丢失 {lost:.0f}s')

    def _enter_error(self, reason: str) -> None:
        self.get_logger().error(f'进入安全错误态: {reason}')
        self.charging_pub.publish(Bool(data=False))
        # 清理充电握手计时器：否则错误→复位→再次泊靠后，旧超时会误杀新握手
        self._charge_request_time = None
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._set_state(ERROR_WAITING_HUMAN, reason + '（等待 /mission/reset）')

    # ------------------------------------------------------------ RViz 文本
    def _publish_status_text(self) -> None:
        arr = MarkerArray()
        for i, text in enumerate((f'SOC: {self.soc:.0%}', f'State: {self.state}')):
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
