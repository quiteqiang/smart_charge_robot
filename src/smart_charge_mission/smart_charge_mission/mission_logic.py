#!/usr/bin/env python3
"""充电任务状态机核心——纯 Python，不依赖 rclpy。

`charge_mission_node.py` 的 rclpy 薄壳负责全部 ROS IO（订阅/服务/定时器/
action client），把外部事件翻译成这里的调用；本核心只维护状态与决策，
副作用以 Effect 列表的形式交回壳执行，所有时间量由壳以 float（秒）注入。

这样设计的收益（docs/improvement_directions.md #4，本项目性价比最高的重构）：
全部转换边——含错误路径、重试耗尽、reset、结果世代竞态——可用 pytest
秒级覆盖，不必跑 12-18 分钟且顺序不可重排的端到端集成测试。

事件方法约定：每个 public 方法返回本次调用产生的 Effect 列表；壳在调用后
立即依次执行。Effect 的 kind 见下方工厂函数。壳提供的 oneshot 定时器到点后
调用对应事件（begin_docking / plan_charge_route / retry_nav / poll_dock /
poll_undock），now 由壳注入。

状态图与 docs/architecture.md 一致：
  IDLE → EXECUTING_TASK ⇄ (低电量) → NAVIGATING_TO_DOCK → PRE_DOCKING → DOCKING
       → CHARGING → UNDOCKING → RESUMING_TASK → EXECUTING_TASK
  任意导航/泊靠失败重试耗尽 → ERROR_WAITING_HUMAN（等待人工 reset）
"""
from __future__ import annotations

import math
from dataclasses import dataclass

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


@dataclass(frozen=True)
class Effect:
    """壳需执行的副作用。

    kind:
      state      (new_state, reason)   状态已变，发布 latched /mission_state
      nav_goal   (name, source)         向 Nav2 下发导航目标
      dock_start ()                     调用 /dock/start 服务
      undock     ()                     调用 /dock/undock 服务
      charging   (active: bool)         发布 /charging_active
      oneshot    (delay_s, event, payload)  壳创建自毁定时器，到点调用同名事件
      cancel_nav ()                     取消当前 Nav2 goal（若有）
    """
    kind: str
    args: tuple = ()


def fx_state(new: str, reason: str) -> Effect:
    return Effect('state', (new, reason))


def fx_nav_goal(name: str, source: str) -> Effect:
    return Effect('nav_goal', (name, source))


def fx_dock_start() -> Effect:
    return Effect('dock_start')


def fx_undock() -> Effect:
    return Effect('undock')


def fx_charging(active: bool) -> Effect:
    return Effect('charging', (active,))


def fx_oneshot(delay_s: float, event: str, payload: tuple = ()) -> Effect:
    return Effect('oneshot', (delay_s, event, payload))


def fx_cancel_nav() -> Effect:
    return Effect('cancel_nav')


class MissionStateMachine:
    """纯 Python 任务状态机。不可重入：壳必须单线程驱动（rclpy 单线程 executor）。"""

    def __init__(self, *, waypoints: dict, task_waypoints: list[str],
                 pre_dock_waypoint: str,
                 low_soc_threshold: float, resume_soc_threshold: float,
                 max_docking_retries: int, max_nav_retries: int,
                 dock_success_timeout_s: float, charge_start_timeout_s: float,
                 tf_loss_timeout_s: float,
                 log=None) -> None:
        self.waypoints = waypoints
        self.task_waypoints = list(task_waypoints)
        self.pre_dock_waypoint = pre_dock_waypoint
        self.low_soc_threshold = float(low_soc_threshold)
        self.resume_soc_threshold = float(resume_soc_threshold)
        self.max_docking_retries = int(max_docking_retries)
        self.max_nav_retries = int(max_nav_retries)
        self.dock_success_timeout_s = float(dock_success_timeout_s)
        self.charge_start_timeout_s = float(charge_start_timeout_s)
        self.tf_loss_timeout_s = float(tf_loss_timeout_s)
        # 日志走注入回调（壳接 get_logger），默认静默便于单测
        self._log = log if log is not None else (lambda level, msg: None)

        self.state = IDLE
        self.soc = 1.0
        self.battery_ok = False
        self.task_queue: list[str] = []
        self.saved_task: str | None = None
        self._low_battery_dwell_id = 0   # LOW_BATTERY 停留世代（孤儿 oneshot 防护）
        self.nav_retries = 0
        self.dock_retries = 0
        # 活动导航目标名（goal handle 由壳持有）；以目标名判定而非 handle：
        # 目标已发出但尚未被接受时 handle 仍为 None，此时航点同样需要保存
        self._active_nav_target: str | None = None
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
        self._dock_wait_start: float | None = None
        self._undock_wait_start: float | None = None
        self._charge_request_time: float | None = None

        self._fx: list[Effect] = []    # 当前事件积累的副作用

    # ------------------------------------------------------------ 内部工具
    def _emit(self, effect: Effect) -> None:
        self._fx.append(effect)

    def _set_state(self, new: str, reason: str = '') -> None:
        if new == self.state:
            return
        self._log('warn', f'状态转换: {self.state} -> {new}  {reason}')
        self.state = new
        self._emit(fx_state(new, reason))

    def _enter_error(self, reason: str) -> None:
        self._log('error', f'进入安全错误态: {reason}')
        self._emit(fx_charging(False))
        # 清理充电握手计时：否则错误→复位→再次泊靠后，旧超时会误杀新握手
        self._charge_request_time = None
        self._emit(fx_cancel_nav())
        self._set_state(ERROR_WAITING_HUMAN, reason + '（等待 /mission/reset）')

    def _advance_task(self) -> None:
        if not self.task_queue:
            self._set_state(IDLE, '全部任务完成')
            return
        name = self.task_queue.pop(0)
        self._set_state(EXECUTING_TASK, f'执行任务 {name}')
        self._emit(fx_nav_goal(name, 'task'))

    # ------------------------------------------------------------ 服务事件
    def start_task(self) -> tuple[bool, str, list[Effect]]:
        """/mission/start_task。返回 (accepted, message, effects)。

        IDLE：开始任务队列；EXECUTING_TASK：抢占当前任务并以新队列重新
        开始——与 /mission/goto 话题拥有同级的抢占能力，修复"正式服务
        接口反而不能抢占、裸话题却可以"的权限倒挂（improvement_directions #7）。
        """
        if self.state not in (IDLE, EXECUTING_TASK):
            return False, f'当前状态 {self.state}，无法开始任务', []
        self._fx = []
        if self.state == EXECUTING_TASK:
            self._log('warn',
                      f'start_task 抢占：取消当前任务 {self._active_nav_target}，'
                      '重新开始任务队列')
            self._emit(fx_cancel_nav())
            self._active_nav_target = None
        self.task_queue = list(self.task_waypoints)
        self._log('info', f'任务队列: {self.task_queue}')
        self._advance_task()
        return True, 'task started', self._fx

    def reset(self) -> tuple[bool, str, list[Effect]]:
        """/mission/reset。仅错误态可复位。"""
        if self.state != ERROR_WAITING_HUMAN:
            return False, '仅错误态可复位', []
        self.saved_task = None
        self.task_queue.clear()
        self.nav_retries = self.dock_retries = 0
        self._fx = []
        self._set_state(IDLE, '人工复位')
        return True, 'reset to IDLE', self._fx

    def goto(self, name: str) -> list[Effect]:
        """/mission/goto 话题。语义：跳转到指定航点。

        IDLE：作为单点任务执行；EXECUTING_TASK：抢占——显式取消当前导航，
        目标替换当前航点（剩余队列保留）。低电量挂起时 saved_task 取自
        _active_nav_target（在 effect 同步执行链中已指向最新意图航点），
        因此任何抢占路径下 saved_task 都指向用户最后意图（#7）。
        """
        self._fx = []
        if name not in self.waypoints:
            self._log('error', f'goto 未知航点: {name}')
            return self._fx
        if self.state not in (IDLE, EXECUTING_TASK):
            self._log('warn', f'goto {name} 被忽略：状态 {self.state}')
            return self._fx
        if self.state == IDLE:
            self.task_queue = [name]
            self._advance_task()
        else:
            if name == self._active_nav_target:
                return self._fx  # 忽略重复目标（连发/双击），防止不必要的抢占
            self._log('warn',
                      f'goto 抢占：导航目标 {self._active_nav_target} -> {name}')
            self._emit(fx_cancel_nav())
            self._emit(fx_nav_goal(name, 'task'))
        return self._fx

    # ------------------------------------------------------------ 导航事件
    def update_config(self, **overrides) -> None:
        """运行时更新可调参数（壳的 on_set_parameters 回调接入 ros2 param set）。

        与旧壳行为一致：阈值/重试/超时/航点配置在下次决策点生效，
        无需重启节点。
        """
        for key, value in overrides.items():
            if not hasattr(self, key):
                raise AttributeError(f'未知配置项: {key}')
            setattr(self, key, value)

    def nav_sent(self, name: str, source: str) -> None:
        """壳已成功把目标发给 action server（原 _send_nav_goal 的赋值点）。"""
        self._active_nav_target = name

    def nav_unavailable(self, name: str, source: str) -> list[Effect]:
        """Nav2 action server 不可用（原 _send_nav_goal 的错误路径）。"""
        self._fx = []
        if source == 'dock':
            self._enter_error('无法规划充电路径：预停靠点无效或 Nav2 不可用')
        else:
            self._enter_error('Nav2 action 不可用')
        return self._fx

    def nav_done(self, success: bool, name: str, source: str) -> list[Effect]:
        """导航结果（含目标被拒绝，拒绝按失败处理）。"""
        self._fx = []
        self._log('warn', f'导航 {name} [{"成功" if success else "失败/取消"}] (来源 {source})')
        # 过期结果防护：目标名与当前活动目标不符（典型：goto/start_task 抢占
        # 后旧目标被取消的迟到结果）则忽略——否则抢占会被误记为导航失败，
        # 触发错误重试甚至进错误态
        if name != self._active_nav_target:
            self._log('warn',
                      f'忽略过期导航结果 {name}（当前目标: {self._active_nav_target}）')
            return self._fx
        self._active_nav_target = None
        # 结果与当前状态不匹配（如低电量已取消并转充电）则忽略，防止误重试
        if source == 'task' and self.state not in (EXECUTING_TASK, RESUMING_TASK):
            return self._fx
        if source == 'dock' and self.state != NAVIGATING_TO_DOCK:
            return self._fx
        if success:
            self.nav_retries = 0
            if source == 'task' and self.state in (EXECUTING_TASK, RESUMING_TASK):
                if self.saved_task is not None:
                    self._log('info', f'已恢复被暂停任务: {self.saved_task}')
                    self.saved_task = None
                self._advance_task()
            elif source == 'dock' and self.state == NAVIGATING_TO_DOCK:
                self._set_state(PRE_DOCKING, '到达充电桩预停靠点')
                self._emit(fx_oneshot(0.5, 'begin_docking'))
            elif source == 'resume_none' and self.state == RESUMING_TASK:
                self._set_state(IDLE, '无被暂停任务，回到空闲')
            return self._fx
        # 失败重试
        self.nav_retries += 1
        if self.nav_retries >= self.max_nav_retries:
            self._enter_error(f'导航 {name} 连续失败 {self.nav_retries} 次')
        else:
            self._log('warn',
                      f'导航重试 {self.nav_retries}/{self.max_nav_retries}: {name}')
            self._emit(fx_oneshot(1.0, 'retry_nav', (name, source)))
        return self._fx

    def retry_nav(self, name: str, source: str) -> list[Effect]:
        '''导航重试前守卫：1s 等待期间状态可能已变（如低电量转充电），
        旧目标的迟到重试不能再发出新导航。'''
        self._fx = []
        expected = {'task': (EXECUTING_TASK, RESUMING_TASK),
                    'dock': (NAVIGATING_TO_DOCK,),
                    'resume_none': (RESUMING_TASK,)}[source]
        if self.state not in expected:
            return self._fx
        self._emit(fx_nav_goal(name, source))
        return self._fx

    # ------------------------------------------------------------ 电池事件
    def battery(self, soc: float, charging: bool, now: float) -> list[Effect]:
        """/battery_state 更新。charging = 电池回报 POWER_SUPPLY_STATUS_CHARGING。"""
        self._fx = []
        if math.isnan(soc) or not 0.0 <= soc <= 1.0:
            self._enter_error(f'电量数据异常: {soc}')
            return self._fx
        self.soc = soc
        self.battery_ok = True
        low = self.low_soc_threshold
        resume = self.resume_soc_threshold

        if soc < low and self.state in (IDLE, EXECUTING_TASK):
            if self.state == EXECUTING_TASK and self._active_nav_target is not None:
                self.saved_task = self._active_nav_target
                # 不显式 cancel：navigate_to_pose 是单目标服务端，下发充电桩目标即抢占并
                # 终结当前任务目标。若在此处 cancel，取消请求会晚 ~30 ms 到达，届时服务端
                # 的当前目标已是 pre_dock —— 取消会连带打掉刚发出的充电桩目标，白白消耗
                # 一次 max_nav_retries（见 issue #6）。被抢占的任务目标结果由 nav_done 的
                # source/state 过滤丢弃，不会触发重试。
                self._log('warn',
                          f'低电量 {soc:.0%}，抢占当前导航并保存任务 {self.saved_task}')
            else:
                self._log('warn', f'低电量 {soc:.0%}，无活动任务，直接前往充电')
            # 保留 task_queue：充电期间无任何路径消费它，恢复被暂停航点后
            # nav_done 会调用 _advance_task 续跑剩余任务。
            # LOW_BATTERY 是真实停留状态（"决策中"）：短暂停留后起导航，
            # 与 docs/architecture.md 状态图一致。
            self._set_state(LOW_BATTERY, f'SOC={soc:.0%} < {low:.0%}')
            self._low_battery_dwell_id += 1
            self._emit(fx_oneshot(1.0, 'plan_charge_route',
                                  (self._low_battery_dwell_id,)))

        if self.state == CHARGING and soc >= resume:
            self._log('info', f'SOC 达到恢复阈值 {soc:.0%} >= {resume:.0%}，结束充电')
            self._set_state(UNDOCKING, '充电完成，离桩')
            self._request_undock(now)

        if self.state == DOCKING and self._charge_request_time is not None:
            # 等待充电握手确认：电池回报 CHARGING
            if charging:
                self._charge_request_time = None
                self._set_state(CHARGING, '充电桩确认开始充电')
            elif now - self._charge_request_time > self.charge_start_timeout_s:
                self._charge_request_time = None
                self._enter_error('充电桩未确认开始充电（超时）')
        return self._fx

    def plan_charge_route(self, dwell_id: int) -> list[Effect]:
        """低电量决策停留结束（oneshot 到点）。

        守卫状态 + 停留世代：期间可能已被看门狗转入错误态，或上一次停留的
        孤儿 oneshot 在新的停留内触发（无世代守卫会提前起导航）。
        """
        self._fx = []
        if self.state != LOW_BATTERY or dwell_id != self._low_battery_dwell_id:
            return self._fx
        self._set_state(NAVIGATING_TO_DOCK, '规划充电路径')
        self.nav_retries = 0
        self.dock_retries = 0
        if self.pre_dock_waypoint not in self.waypoints:
            self._enter_error('无法规划充电路径：预停靠点无效')
            return self._fx
        self._emit(fx_nav_goal(self.pre_dock_waypoint, 'dock'))
        return self._fx

    # ------------------------------------------------------------ 泊靠/离桩结果
    def dock_result(self, sequence: int, success: bool) -> list[Effect]:
        """/docking_success 更新（泊靠与离桩互斥进行：两个 pending 都更新，
        轮询时按世代基线各取所需）。"""
        self._fx = []
        if sequence < self._last_result_seq:
            # 序号回退 = 控制器重启、序号空间重置：旧基线失效，进入新世代。
            # （重启后 in-flight 的少量旧消息可能再触发一次，方向是 fail-safe：
            # 结果按"新"被接受而非被永久拒绝，且泊靠结果仍由控制器状态机背书）
            self._result_epoch += 1
        self._last_result_seq = sequence
        self.pending_dock_result = (self._result_epoch, sequence, success)
        self.pending_undock_result = (self._result_epoch, sequence, success)
        return self._fx

    @staticmethod
    def _result_newer(result: tuple[int, int, bool], req_epoch: int, req_seq: int) -> bool:
        '''结果是否晚于请求基线（跨世代或同世代序号更新）。'''
        return result[0] > req_epoch or (result[0] == req_epoch and result[1] > req_seq)

    # ------------------------------------------------------------ 泊靠
    def begin_docking(self, now: float) -> list[Effect]:
        """PRE_DOCKING 停留结束，发起第 dock_retries+1 次泊靠。"""
        self._fx = []
        if self.state != PRE_DOCKING:
            return self._fx
        self._set_state(DOCKING, f'第 {self.dock_retries + 1} 次泊靠尝试')
        self.pending_dock_result = None
        self._dock_req_epoch = self._result_epoch
        self._dock_req_seq = self._last_result_seq
        self._dock_wait_start = now
        self._emit(fx_dock_start())
        return self._fx

    def dock_start_response(self, ok: bool, reason: str = '') -> list[Effect]:
        """/dock/start 服务响应（ok=False 涵盖服务不可用/拒绝/调用异常）。

        状态守卫：响应在途期间看门狗/电池异常可能已把状态机转入错误态，
        迟到的失败响应不得把机器从人工监督的错误态拖回导航。
        """
        self._fx = []
        if self.state != DOCKING:
            if not ok:
                self._log('warn',
                          f'忽略迟到/错序的泊靠启动响应({reason})，当前状态 {self.state}')
            return self._fx
        if not ok:
            self._dock_failed(reason)
        else:
            self._emit(fx_oneshot(0.5, 'poll_dock'))
        return self._fx

    def poll_dock(self, now: float) -> list[Effect]:
        """轮询泊靠结果（oneshot 自续，直到结果或超时）。"""
        self._fx = []
        if self.state != DOCKING:
            return self._fx
        result = self.pending_dock_result
        # 世代校验：同世代且序号不晚于请求基线的是 latched 旧值（重订阅 replay），忽略
        if result is not None and \
                self._result_newer(result, self._dock_req_epoch, self._dock_req_seq):
            if result[2]:
                self._dock_succeeded(now)
            else:
                self._dock_failed('泊靠控制器报告失败')
            return self._fx
        # 总超时保护（控制器自身有超时，这里兜底防死等）
        if self._dock_wait_start is not None and \
                now - self._dock_wait_start > self.dock_success_timeout_s:
            self._dock_wait_start = None
            self._dock_failed('等待泊靠结果超时')
            return self._fx
        self._emit(fx_oneshot(0.5, 'poll_dock'))
        return self._fx

    def _dock_succeeded(self, now: float) -> None:
        self.dock_retries = 0
        self._log('info', '泊靠成功，请求开始充电')
        self._emit(fx_charging(True))
        self._charge_request_time = now   # 等电池节点回报 CHARGING

    def _dock_failed(self, reason: str) -> None:
        self._emit(fx_charging(False))
        self.dock_retries += 1
        self._log('error',
                  f'泊靠失败({reason})，第 {self.dock_retries}/{self.max_docking_retries} 次')
        if self.dock_retries >= self.max_docking_retries:
            self._enter_error(f'泊靠重试 {self.max_docking_retries} 次均失败: {reason}')
            return
        # 退回预停靠点重新尝试
        self._set_state(NAVIGATING_TO_DOCK,
                        f'泊靠失败重试 {self.dock_retries}/{self.max_docking_retries}')
        self._emit(fx_nav_goal(self.pre_dock_waypoint, 'dock'))

    # ------------------------------------------------------------ 离桩/恢复
    def _request_undock(self, now: float) -> None:
        self._emit(fx_charging(False))
        self.pending_undock_result = None
        self._undock_req_epoch = self._result_epoch
        self._undock_req_seq = self._last_result_seq
        self._undock_wait_start = now
        self._emit(fx_undock())
        self._emit(fx_oneshot(0.5, 'poll_undock'))

    def undock_response(self, ok: bool, reason: str = '') -> list[Effect]:
        """/dock/undock 服务响应。

        状态守卫同 dock_start_response：人工复位后迟到的失败响应不得
        把机器从 IDLE 弹回错误态。
        """
        self._fx = []
        if self.state != UNDOCKING:
            if not ok:
                self._log('warn',
                          f'忽略迟到/错序的离桩响应({reason})，当前状态 {self.state}')
            return self._fx
        if not ok:
            self._enter_error(reason)
        return self._fx

    def poll_undock(self, now: float) -> list[Effect]:
        """轮询离桩结果（oneshot 自续，直到结果或超时）。"""
        self._fx = []
        if self.state != UNDOCKING:
            return self._fx
        # 超时兜底：控制器可能在受理离桩后、发布结果前死亡，无超时则永久挂起。
        # 注意与原 dock 轮询的顺序差异：这里先查超时再看结果，保持既有行为。
        if self._undock_wait_start is not None and \
                now - self._undock_wait_start > self.dock_success_timeout_s:
            self._undock_wait_start = None
            self._enter_error('等待离桩结果超时')
            return self._fx
        result = self.pending_undock_result
        # 世代校验：同 poll_dock，忽略 latched 旧值
        if result is not None and \
                self._result_newer(result, self._undock_req_epoch, self._undock_req_seq):
            if result[2]:
                self._resume_task()
            else:
                self._enter_error('离桩失败')
            return self._fx
        self._emit(fx_oneshot(0.5, 'poll_undock'))
        return self._fx

    def _resume_task(self) -> None:
        if self.saved_task is not None:
            self._set_state(RESUMING_TASK, f'返回被暂停任务 {self.saved_task}')
            self._emit(fx_nav_goal(self.saved_task, 'task'))
        else:
            self._set_state(RESUMING_TASK, '无被暂停任务')
            self._emit(fx_nav_goal(self.pre_dock_waypoint, 'resume_none'))

    # ------------------------------------------------------------ 看门狗
    def tf_lost(self, lost_s: float) -> list[Effect]:
        """壳测得 map->base_link TF 丢失超时。壳负责状态前置过滤与计时。"""
        self._fx = []
        self._enter_error(f'定位失效: map->base_link TF 丢失 {lost_s:.0f}s')
        return self._fx
