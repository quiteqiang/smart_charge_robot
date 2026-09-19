#!/usr/bin/env python3
"""充电任务状态机 —— rclpy 薄壳。

所有决策逻辑在 mission_core.MissionCore（不依赖 rclpy 的纯 Python 类）中；
本节点只做 IO：把 rclpy 回调翻译成 mission_types.Event，喂给
core.handle(event)，再按顺序执行返回的 mission_types.Command 列表
（发送导航目标、调用泊靠/离桩服务、发布状态、调度定时器……）。

状态图、设计要点见 mission_core.py 顶部docstring 与
docs/architecture.md。
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

from smart_charge_mission.mission_core import MissionCore
from smart_charge_mission.mission_types import (
    BatteryReading,
    CallDockService,
    CancelNavGoal,
    DockResultReceived,
    DockServiceResponse,
    GotoRequested,
    Log,
    MissionState,
    NavGoalResponse,
    NavResult,
    NavServerUnavailable,
    PublishChargingActive,
    PublishState,
    ResetRequested,
    ScheduleTimer,
    SendNavGoal,
    StartTaskRequested,
    TfCheckResult,
    TimerFired,
    TriggerAck,
)


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

        with open(self.get_parameter('waypoints_file').value, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        frame = data.get('frame', 'map')
        waypoints = {
            name: (float(wp['x']), float(wp['y']), float(wp['yaw']))
            for name, wp in data['waypoints'].items()
        }

        # MissionCore config is a frozen snapshot taken at construction time —
        # unlike the old inline get_parameter(...) reads, a runtime
        # `ros2 param set` on any of these no longer takes effect without a
        # node restart. No existing integration scenario exercises live
        # reconfiguration of the mission node's own parameters.
        self.core = MissionCore(
            waypoints=waypoints,
            frame=frame,
            task_waypoints=list(self.get_parameter('task_waypoints').value),
            pre_dock_waypoint=self.get_parameter('pre_dock_waypoint').value,
            low_soc_threshold=self.get_parameter('low_soc_threshold').value,
            resume_soc_threshold=self.get_parameter('resume_soc_threshold').value,
            max_nav_retries=self.get_parameter('max_nav_retries').value,
            max_docking_retries=self.get_parameter('max_docking_retries').value,
            dock_success_timeout_s=self.get_parameter('dock_success_timeout_s').value,
            charge_start_timeout_s=self.get_parameter('charge_start_timeout_s').value,
            tf_loss_timeout_s=self.get_parameter('tf_loss_timeout_s').value,
        )

        # request_id/timer_id -> live rclpy object, populated as commands are executed.
        self._nav_goal_handles: dict[int, object] = {}
        self._timers: dict[int, object] = {}

        # rclpy.spin(node) below uses the default SingleThreadedExecutor, so
        # MissionCore.handle() is never called concurrently. This callback
        # group is inert in practice; if the executor ever becomes
        # multithreaded, core.handle() needs a lock.
        group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.dock_start = self.create_client(Trigger, '/dock/start')
        self.dock_undock = self.create_client(Trigger, '/dock/undock')

        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        # TRANSIENT_LOCAL 与发布端匹配：重订阅时能拿到 latched 基线/旧结果，
        # 由 MissionCore 的世代校验负责丢弃过期值
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
        self.state_pub.publish(String(data=self.core.state.value))
        self.get_logger().info('充电任务状态机就绪 (IDLE)，等待 /mission/start_task')

    # ------------------------------------------------------------ core plumbing
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _dispatch(self, event) -> TriggerAck | None:
        return self._execute(self.core.handle(event))

    def _execute(self, cmds) -> TriggerAck | None:
        ack: TriggerAck | None = None
        for cmd in cmds:
            if isinstance(cmd, PublishState):
                self.state_pub.publish(String(data=cmd.state.value))
            elif isinstance(cmd, PublishChargingActive):
                self.charging_pub.publish(Bool(data=cmd.active))
            elif isinstance(cmd, SendNavGoal):
                self._exec_send_nav_goal(cmd)
            elif isinstance(cmd, CancelNavGoal):
                self._exec_cancel_nav_goal(cmd)
            elif isinstance(cmd, CallDockService):
                self._exec_call_dock_service(cmd)
            elif isinstance(cmd, ScheduleTimer):
                self._exec_schedule_timer(cmd)
            elif isinstance(cmd, TriggerAck):
                ack = cmd
            elif isinstance(cmd, Log):
                getattr(self.get_logger(), cmd.level)(cmd.message)
        return ack

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _exec_send_nav_goal(self, cmd: SendNavGoal) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = cmd.frame
        pose.pose.position.x = cmd.x
        pose.pose.position.y = cmd.y
        pose.pose.orientation = self._yaw_to_quat(cmd.yaw)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action 不可用')
            self._dispatch(NavServerUnavailable(now=self._now(), request_id=cmd.request_id))
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_nav_goal_response_cb(f, cmd.request_id))

    def _on_nav_goal_response_cb(self, future, request_id: int) -> None:
        handle = future.result()
        if not handle.accepted:
            self._dispatch(NavGoalResponse(now=self._now(), request_id=request_id, accepted=False))
            return
        self._nav_goal_handles[request_id] = handle
        self._dispatch(NavGoalResponse(now=self._now(), request_id=request_id, accepted=True))
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result_cb(f, request_id))

    def _on_nav_result_cb(self, future, request_id: int) -> None:
        self._nav_goal_handles.pop(request_id, None)
        try:
            result = future.result()
            ok = result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception as exc:  # noqa: BLE001 - action 结果异常统一按失败处理
            self.get_logger().error(f'导航结果异常: {exc}')
            ok = False
        self._dispatch(NavResult(now=self._now(), request_id=request_id, success=ok))

    def _exec_cancel_nav_goal(self, cmd: CancelNavGoal) -> None:
        handle = self._nav_goal_handles.pop(cmd.request_id, None)
        if handle is not None:
            handle.cancel_goal_async()

    def _exec_call_dock_service(self, cmd: CallDockService) -> None:
        client = self.dock_start if cmd.kind == 'start' else self.dock_undock
        if not client.wait_for_service(timeout_sec=5.0):
            self._dispatch(DockServiceResponse(
                now=self._now(), kind=cmd.kind, accepted=False,
                detail=f'{cmd.kind} 服务不可用'))
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_dock_service_response_cb(f, cmd.kind))

    def _on_dock_service_response_cb(self, future, kind: str) -> None:
        try:
            ok = future.result().success
            detail = '' if ok else f'{kind} 服务拒绝'
        except Exception as exc:  # noqa: BLE001
            ok = False
            detail = f'{kind} 服务调用异常: {exc}'
        self._dispatch(DockServiceResponse(now=self._now(), kind=kind, accepted=ok, detail=detail))

    def _exec_schedule_timer(self, cmd: ScheduleTimer) -> None:
        def fire() -> None:
            timer = self._timers.pop(cmd.timer_id, None)
            if timer is not None:
                self.destroy_timer(timer)
            self._dispatch(TimerFired(now=self._now(), timer_id=cmd.timer_id))
        self._timers[cmd.timer_id] = self.create_timer(cmd.delay_s, fire)

    # ------------------------------------------------------------ 服务/话题回调
    def _on_start_task(self, _, response: Trigger.Response) -> Trigger.Response:
        ack = self._dispatch(StartTaskRequested(now=self._now()))
        if ack is not None:
            response.success = ack.success
            response.message = ack.message
        return response

    def _on_reset(self, _, response: Trigger.Response) -> Trigger.Response:
        ack = self._dispatch(ResetRequested(now=self._now()))
        if ack is not None:
            response.success = ack.success
            response.message = ack.message
        return response

    def _on_goto(self, msg: String) -> None:
        self._dispatch(GotoRequested(now=self._now(), waypoint=msg.data.strip()))

    def _on_battery(self, msg: BatteryState) -> None:
        charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self._dispatch(BatteryReading(now=self._now(), percentage=msg.percentage, charging=charging))

    def _on_dock_result(self, msg: DockResult) -> None:
        self._dispatch(DockResultReceived(now=self._now(), sequence=msg.sequence, success=msg.success))

    def _on_docking_status(self, msg: String) -> None:
        self.get_logger().info(f'[泊靠] {msg.data}')

    # ------------------------------------------------------------ 看门狗
    def _watchdog(self) -> None:
        # 定位看门狗：map->base_link TF 丢失视为定位失效。状态过滤留在 shell
        # 侧（而非 core 里）：保持与原实现完全一致的"无宽限期"时序——长时间
        # 停留在 IDLE/ERROR 期间不做 TF 查询，离开这些状态后第一次 tick 才
        # 开始真正判定丢失与否。
        if self.core.state in (MissionState.IDLE, MissionState.ERROR_WAITING_HUMAN):
            return
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                            timeout=Duration(seconds=0.2))
            ok = True
        except TransformException:
            ok = False
        self._dispatch(TfCheckResult(now=self._now(), ok=ok))

    # ------------------------------------------------------------ RViz 文本
    def _publish_status_text(self) -> None:
        arr = MarkerArray()
        for i, text in enumerate((f'SOC: {self.core.soc:.0%}', f'State: {self.core.state.value}')):
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
