"""Pure-Python mission state machine (no rclpy imports).

This is a 1:1 behavioral port of the decision logic that used to live inline
in charge_mission_node.py's rclpy callbacks. It is deliberately faithful,
bugs and all (see the ISSUE #4 note below) — charge_mission_node.py is now a
thin shell that translates rclpy callbacks into Event objects, calls
MissionCore.handle(event), and executes the returned Command objects in
order.

Two known cancel-race issues from the ROS node era are represented here:
  - ISSUE #6 (fixed by design, pinned by a test): the low-battery path never
    emits CancelNavGoal for the preempted task goal — it relies entirely on
    Nav2's single-goal-preemption semantics. See _on_battery.
  - ISSUE #4 (still open, ported as-is): _on_nav_goal_response adopts an
    accepted goal as "the" tracked goal unconditionally, with no check that
    it still corresponds to the currently-active request. A late accept for
    an already-superseded request can clobber this, so a subsequent
    _enter_error() may cancel the wrong (already-preempted) goal. See
    _on_nav_goal_response and tests/test_mission_core.py's xfail test.
"""
from __future__ import annotations

import math

from smart_charge_mission.mission_types import (
    BatteryReading,
    CallDockService,
    CancelNavGoal,
    Command,
    DockResultReceived,
    DockServiceResponse,
    Event,
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

_NAV_RETRY_EXPECTED_STATES = {
    'task': (MissionState.EXECUTING_TASK, MissionState.RESUMING_TASK),
    'dock': (MissionState.NAVIGATING_TO_DOCK,),
    'resume_none': (MissionState.RESUMING_TASK,),
}


class MissionCore:
    def __init__(
        self,
        waypoints: dict[str, tuple[float, float, float]],
        frame: str,
        task_waypoints: list[str],
        pre_dock_waypoint: str,
        low_soc_threshold: float,
        resume_soc_threshold: float,
        max_nav_retries: int,
        max_docking_retries: int,
        dock_success_timeout_s: float,
        charge_start_timeout_s: float,
        tf_loss_timeout_s: float,
    ) -> None:
        self._waypoints = dict(waypoints)
        self._frame = frame
        self._task_waypoints = list(task_waypoints)
        self._pre_dock_waypoint = pre_dock_waypoint
        self._low_soc_threshold = low_soc_threshold
        self._resume_soc_threshold = resume_soc_threshold
        self._max_nav_retries = max_nav_retries
        self._max_docking_retries = max_docking_retries
        self._dock_success_timeout_s = dock_success_timeout_s
        self._charge_start_timeout_s = charge_start_timeout_s
        self._tf_loss_timeout_s = tf_loss_timeout_s

        self.state = MissionState.IDLE
        self.soc = 1.0
        self.task_queue: list[str] = []
        self.saved_task: str | None = None
        self.nav_retries = 0
        self.dock_retries = 0

        # (epoch, sequence, success) — see _result_newer.
        self.pending_dock_result: tuple[int, int, bool] | None = None
        self.pending_undock_result: tuple[int, int, bool] | None = None
        self._result_epoch = 0
        self._last_result_seq = 0
        self._dock_req_epoch = 0
        self._dock_req_seq = 0
        self._undock_req_epoch = 0
        self._undock_req_seq = 0

        self._active_nav_target: str | None = None
        # Mirrors charge_mission_node's self._nav_goal_handle. ISSUE #4:
        # assigned unconditionally on NavGoalResponse(accepted=True), with no
        # check that request_id is still the active one. Preserved as-is.
        self._tracked_nav_request_id: int | None = None
        self._nav_requests: dict[int, tuple[str, str]] = {}
        self._next_request_id = 1

        self._dock_wait_start: float | None = None
        self._undock_wait_start: float | None = None
        self._last_tf_ok_time: float | None = None
        self._charge_request_time: float | None = None

        # Generation guard for the LOW_BATTERY dwell timer only (see
        # _on_dwell_fired) — matches the original's _low_battery_dwell_id.
        # Every other timer site is guarded by state alone, same as today.
        self._dwell_timer_id: int | None = None

        self._next_timer_id = 1
        # timer_id -> (site, payload). The payload carries per-timer context
        # that the original captured in each _oneshot's closure (the nav
        # retry's waypoint/source), so two overlapping retries can't read
        # each other's target.
        self._pending_timers: dict[int, tuple[str, object]] = {}

    # ------------------------------------------------------------ entrypoint
    def handle(self, event: Event) -> list[Command]:
        cmds: list[Command] = []
        if isinstance(event, StartTaskRequested):
            self._on_start_task(cmds, event.now)
        elif isinstance(event, ResetRequested):
            self._on_reset(cmds, event.now)
        elif isinstance(event, GotoRequested):
            self._on_goto(cmds, event.now, event.waypoint)
        elif isinstance(event, BatteryReading):
            self._on_battery(cmds, event.now, event.percentage, event.charging)
        elif isinstance(event, NavGoalResponse):
            self._on_nav_goal_response(cmds, event.now, event.request_id, event.accepted)
        elif isinstance(event, NavResult):
            self._on_nav_done(cmds, event.now, event.request_id, event.success)
        elif isinstance(event, NavServerUnavailable):
            self._on_nav_server_unavailable(cmds, event.now, event.request_id)
        elif isinstance(event, DockResultReceived):
            self._on_dock_result(event.sequence, event.success)
        elif isinstance(event, DockServiceResponse):
            self._on_dock_service_response(cmds, event.now, event.kind, event.accepted, event.detail)
        elif isinstance(event, TimerFired):
            self._on_timer_fired(cmds, event.now, event.timer_id)
        elif isinstance(event, TfCheckResult):
            self._on_tf_check_result(cmds, event.now, event.ok)
        return cmds

    # ------------------------------------------------------------ helpers
    def _set_state(self, cmds: list[Command], new: MissionState, reason: str = '') -> None:
        if new == self.state:
            return
        cmds.append(Log('warn', f'state transition: {self.state.value} -> {new.value}  {reason}'))
        self.state = new
        cmds.append(PublishState(new))

    def _schedule_timer(self, cmds: list[Command], delay_s: float, kind: str,
                        payload: object = None) -> int:
        timer_id = self._next_timer_id
        self._next_timer_id += 1
        self._pending_timers[timer_id] = (kind, payload)
        cmds.append(ScheduleTimer(timer_id, delay_s))
        return timer_id

    def _try_send_nav_goal(self, cmds: list[Command], waypoint: str, source: str, now: float) -> bool:
        """Mirrors _send_nav_goal's pose-lookup half.

        Nav2-server-availability is inherently async here (see
        NavServerUnavailable) — unlike the original, which blocked on
        wait_for_server before returning, this always emits SendNavGoal once
        the waypoint is known and lets the shell report unavailability later.
        The return value only reflects whether the waypoint was known, which
        is exactly what most callers ignore today (see R6 in the plan) and
        one (_on_dwell_fired) checks.
        """
        pose = self._waypoints.get(waypoint)
        if pose is None:
            cmds.append(Log('error', f'unknown waypoint: {waypoint}'))
            return False
        x, y, yaw = pose
        request_id = self._next_request_id
        self._next_request_id += 1
        self._nav_requests[request_id] = (waypoint, source)
        self._active_nav_target = waypoint
        self._tracked_nav_request_id = None
        cmds.append(SendNavGoal(request_id, waypoint, x, y, yaw, self._frame, source))
        cmds.append(Log('info', f'[{source}] navigate to {waypoint} ({x:.1f}, {y:.1f})'))
        return True

    def _advance_task(self, cmds: list[Command], now: float) -> None:
        if not self.task_queue:
            self._set_state(cmds, MissionState.IDLE, 'all tasks complete')
            return
        name = self.task_queue.pop(0)
        self._set_state(cmds, MissionState.EXECUTING_TASK, f'executing task {name}')
        self._try_send_nav_goal(cmds, name, 'task', now)

    def _enter_error(self, cmds: list[Command], now: float, reason: str) -> None:
        cmds.append(Log('error', f'entering safety error state: {reason}'))
        cmds.append(PublishChargingActive(False))
        # Clear the charge handshake timer: otherwise error -> reset -> a new
        # docking attempt could have its fresh handshake killed by the old timeout.
        self._charge_request_time = None
        if self._tracked_nav_request_id is not None:
            cmds.append(CancelNavGoal(self._tracked_nav_request_id))
            self._tracked_nav_request_id = None
        self._set_state(cmds, MissionState.ERROR_WAITING_HUMAN, reason + ' (awaiting /mission/reset)')

    # ------------------------------------------------------------ service/topic requests
    def _on_start_task(self, cmds: list[Command], now: float) -> None:
        if self.state != MissionState.IDLE:
            cmds.append(TriggerAck(False, f'cannot start task in state {self.state.value}'))
            return
        self.task_queue = list(self._task_waypoints)
        cmds.append(Log('info', f'task queue: {self.task_queue}'))
        self._advance_task(cmds, now)
        cmds.append(TriggerAck(True, 'task started'))

    def _on_reset(self, cmds: list[Command], now: float) -> None:
        if self.state != MissionState.ERROR_WAITING_HUMAN:
            cmds.append(TriggerAck(False, 'can only reset from the error state'))
            return
        self.saved_task = None
        self.task_queue.clear()
        self.nav_retries = self.dock_retries = 0
        self._set_state(cmds, MissionState.IDLE, 'manual reset')
        cmds.append(TriggerAck(True, 'reset to IDLE'))

    def _on_goto(self, cmds: list[Command], now: float, name: str) -> None:
        if name not in self._waypoints:
            cmds.append(Log('error', f'goto unknown waypoint: {name}'))
            return
        if self.state not in (MissionState.IDLE, MissionState.EXECUTING_TASK):
            cmds.append(Log('warn', f'goto {name} ignored: state {self.state.value}'))
            return
        if self.state == MissionState.IDLE:
            self.task_queue = [name]
            self._advance_task(cmds, now)
        else:
            if name == self._active_nav_target:
                return  # dedupe a repeated goto (double click), avoid needless preemption
            self._try_send_nav_goal(cmds, name, 'task', now)

    # ------------------------------------------------------------ battery
    def _on_battery(self, cmds: list[Command], now: float, percentage: float, charging: bool) -> None:
        if math.isnan(percentage) or not 0.0 <= percentage <= 1.0:
            self._enter_error(cmds, now, f'invalid SOC reading: {percentage}')
            return
        self.soc = percentage

        if self.soc < self._low_soc_threshold and self.state in (MissionState.IDLE, MissionState.EXECUTING_TASK):
            # Pause the current task: save its waypoint; the current nav goal is
            # preempted by whichever charger goal gets sent next.
            #判定用 _active_nav_target 而非 handle：目标已发出但尚未被接受时
            # handle 仍为 None，此时航点同样需要保存。
            if self.state == MissionState.EXECUTING_TASK and self._active_nav_target is not None:
                self.saved_task = self._active_nav_target
                # ISSUE #6 (fixed by design — do not reintroduce a cancel here):
                # navigate_to_pose is a single-goal action server, so sending the
                # charger goal next implicitly preempts whatever's in flight. An
                # explicit cancel_goal_async() here would arrive ~30ms late, by
                # which point the server's current goal is already the charger
                # goal — the cancel would kill THAT instead, wasting a retry.
                # The preempted task goal's eventual result is discarded by the
                # source/state filter in _on_nav_done.
                cmds.append(Log('warn', f'low battery {self.soc:.0%}, preempting nav, saved task {self.saved_task}'))
            else:
                cmds.append(Log('warn', f'low battery {self.soc:.0%}, no active task, heading to charge'))
            # task_queue is preserved: nothing on the charge path consumes it;
            # once the saved waypoint is resumed, _on_nav_done's _advance_task
            # call continues the remaining queue.
            self._set_state(cmds, MissionState.LOW_BATTERY, f'SOC={self.soc:.0%} < {self._low_soc_threshold:.0%}')
            self._dwell_timer_id = self._schedule_timer(cmds, 1.0, 'dwell')

        if self.state == MissionState.CHARGING and self.soc >= self._resume_soc_threshold:
            cmds.append(Log('info', f'SOC reached resume threshold {self.soc:.0%} >= {self._resume_soc_threshold:.0%}'))
            self._set_state(cmds, MissionState.UNDOCKING, 'charging complete, undocking')
            self._request_undock(cmds, now)

        if self.state == MissionState.DOCKING and self._charge_request_time is not None:
            if charging:
                self._charge_request_time = None
                self._set_state(cmds, MissionState.CHARGING, 'charger confirmed charging')
            elif now - self._charge_request_time > self._charge_start_timeout_s:
                self._charge_request_time = None
                self._enter_error(cmds, now, 'charger did not confirm charging (timeout)')

    # ------------------------------------------------------------ nav
    def _on_nav_goal_response(self, cmds: list[Command], now: float, request_id: int, accepted: bool) -> None:
        if not accepted:
            self._on_nav_done(cmds, now, request_id, False)
            return
        # ISSUE #4 (open, ported as-is): adopt this response unconditionally,
        # with no check that request_id is still the currently-active
        # request. A late accept for an already-superseded goal can clobber
        # this, so a later _enter_error() may cancel the wrong goal.
        self._tracked_nav_request_id = request_id

    def _on_nav_server_unavailable(self, cmds: list[Command], now: float, request_id: int) -> None:
        self._nav_requests.pop(request_id, None)
        self._enter_error(cmds, now, 'Nav2 action server unavailable')

    def _on_nav_done(self, cmds: list[Command], now: float, request_id: int, success: bool) -> None:
        name, source = self._nav_requests.pop(request_id, (None, None))
        # Unconditional clear, even for a stale/irrelevant result — matches
        # the original, which clears bookkeeping before checking relevance.
        self._tracked_nav_request_id = None
        self._active_nav_target = None
        cmds.append(Log('warn', f'nav {name} [{"succeeded" if success else "failed/cancelled"}] (source {source})'))
        if source is None:
            # Unknown/already-consumed request id: there is no waypoint or
            # source to route on. Not reachable from the current shell, but
            # falling through would retry a phantom goal and then raise a
            # KeyError out of handle(), killing the node's callback.
            return
        if source == 'task' and self.state not in (MissionState.EXECUTING_TASK, MissionState.RESUMING_TASK):
            return
        if source == 'dock' and self.state != MissionState.NAVIGATING_TO_DOCK:
            return
        if success:
            self.nav_retries = 0
            if source == 'task' and self.state in (MissionState.EXECUTING_TASK, MissionState.RESUMING_TASK):
                if self.saved_task is not None:
                    cmds.append(Log('info', f'resumed previously-paused task: {self.saved_task}'))
                    self.saved_task = None
                self._advance_task(cmds, now)
            elif source == 'dock' and self.state == MissionState.NAVIGATING_TO_DOCK:
                self._set_state(cmds, MissionState.PRE_DOCKING, 'reached pre-dock waypoint')
                self._schedule_timer(cmds, 0.5, 'pre_docking')
            elif source == 'resume_none' and self.state == MissionState.RESUMING_TASK:
                self._set_state(cmds, MissionState.IDLE, 'no paused task, returning to idle')
            return
        # Failed / cancelled — retry with backoff, note no per-source state
        # guard exists here beyond the two early returns above (a stale
        # 'resume_none' result is not filtered — preserved as-is).
        self.nav_retries += 1
        if self.nav_retries >= self._max_nav_retries:
            self._enter_error(cmds, now, f'nav {name} failed {self.nav_retries} times in a row')
        else:
            cmds.append(Log('warn', f'nav retry {self.nav_retries}/{self._max_nav_retries}: {name}'))
            self._schedule_timer(cmds, 1.0, 'nav_retry', payload=(name, source))

    def _on_nav_retry_fired(self, cmds: list[Command], now: float,
                            payload: tuple[str, str]) -> None:
        name, source = payload
        if self.state not in _NAV_RETRY_EXPECTED_STATES[source]:
            return  # state changed during the 1s backoff (e.g. low battery); drop it
        self._try_send_nav_goal(cmds, name, source, now)

    # ------------------------------------------------------------ docking
    @staticmethod
    def _result_newer(result: tuple[int, int, bool], req_epoch: int, req_seq: int) -> bool:
        """Is `result` newer than the (epoch, sequence) baseline captured at request time?"""
        return result[0] > req_epoch or (result[0] == req_epoch and result[1] > req_seq)

    def _on_dock_result(self, sequence: int, success: bool) -> None:
        # Docking and undocking are mutually exclusive in time: both pending_*
        # slots are updated from every result; each poller applies its own
        # request-time baseline to pick out the one it's waiting for.
        if sequence < self._last_result_seq:
            # Sequence went backwards: controller restarted / sequence space
            # reset. Bump the epoch rather than reject future results forever
            # (fail-safe towards accepting genuine results over rejecting them).
            self._result_epoch += 1
        self._last_result_seq = sequence
        self.pending_dock_result = (self._result_epoch, sequence, success)
        self.pending_undock_result = (self._result_epoch, sequence, success)

    def _on_dwell_fired(self, cmds: list[Command], now: float, timer_id: int) -> None:
        # Guarded by state AND timer identity: an orphaned dwell timer from an
        # earlier LOW_BATTERY episode (interrupted by an error before it
        # fired) must not prematurely end a *new* dwell that happens to also
        # be LOW_BATTERY.
        if self.state != MissionState.LOW_BATTERY or timer_id != self._dwell_timer_id:
            return
        self._set_state(cmds, MissionState.NAVIGATING_TO_DOCK, 'planning charge route')
        self.nav_retries = 0
        self.dock_retries = 0
        if not self._try_send_nav_goal(cmds, self._pre_dock_waypoint, 'dock', now):
            self._enter_error(cmds, now, 'cannot plan charge route: pre-dock waypoint unknown')

    def _begin_docking(self, cmds: list[Command], now: float) -> None:
        if self.state != MissionState.PRE_DOCKING:
            return
        self._set_state(cmds, MissionState.DOCKING, f'docking attempt {self.dock_retries + 1}')
        self.pending_dock_result = None
        self._dock_req_epoch = self._result_epoch
        self._dock_req_seq = self._last_result_seq
        self._dock_wait_start = now
        cmds.append(CallDockService('start'))

    def _poll_dock_result(self, cmds: list[Command], now: float) -> None:
        if self.state != MissionState.DOCKING:
            return
        result = self.pending_dock_result
        if result is not None and self._result_newer(result, self._dock_req_epoch, self._dock_req_seq):
            if result[2]:
                self._dock_succeeded(cmds, now)
            else:
                self._dock_failed(cmds, now, 'dock controller reported failure')
            return
        if now - self._dock_wait_start > self._dock_success_timeout_s:
            self._dock_wait_start = None
            self._dock_failed(cmds, now, 'timed out waiting for dock result')
            return
        self._schedule_timer(cmds, 0.5, 'dock_poll')

    def _dock_succeeded(self, cmds: list[Command], now: float) -> None:
        self.dock_retries = 0
        cmds.append(Log('info', 'dock succeeded, requesting charge start'))
        cmds.append(PublishChargingActive(True))
        self._charge_request_time = now  # awaiting battery node's CHARGING confirmation

    def _dock_failed(self, cmds: list[Command], now: float, reason: str) -> None:
        cmds.append(PublishChargingActive(False))
        self.dock_retries += 1
        cmds.append(Log('error', f'dock failed ({reason}), attempt {self.dock_retries}/{self._max_docking_retries}'))
        if self.dock_retries >= self._max_docking_retries:
            self._enter_error(cmds, now, f'dock retries exhausted ({self._max_docking_retries}): {reason}')
            return
        self._set_state(cmds, MissionState.NAVIGATING_TO_DOCK, f'retrying dock {self.dock_retries}/{self._max_docking_retries}')
        # Return value intentionally unchecked here, matching the original
        # (unlike _on_dwell_fired, which does check it).
        self._try_send_nav_goal(cmds, self._pre_dock_waypoint, 'dock', now)

    def _on_dock_service_response(
        self, cmds: list[Command], now: float, kind: str, accepted: bool, detail: str = ''
    ) -> None:
        if kind == 'start':
            if not accepted:
                self._dock_failed(cmds, now, detail or 'dock service rejected/unavailable')
                return
            self._schedule_timer(cmds, 0.5, 'dock_poll')
        elif kind == 'undock':
            if not accepted:
                self._enter_error(cmds, now, detail or 'undock rejected/unavailable')

    # ------------------------------------------------------------ undocking / resume
    def _request_undock(self, cmds: list[Command], now: float) -> None:
        cmds.append(PublishChargingActive(False))
        self.pending_undock_result = None
        self._undock_req_epoch = self._result_epoch
        self._undock_req_seq = self._last_result_seq
        self._undock_wait_start = now
        cmds.append(CallDockService('undock'))
        # Unlike docking, the poll loop starts immediately rather than only
        # after the service confirms acceptance — matches the original's
        # _request_undock, which schedules _poll_undock_result right after
        # call_async() without waiting for its done-callback.
        self._schedule_timer(cmds, 0.5, 'undock_poll')

    def _poll_undock_result(self, cmds: list[Command], now: float) -> None:
        if self.state != MissionState.UNDOCKING:
            return
        # Timeout is checked first here (matches the original _poll_undock_result,
        # unlike _poll_dock_result which checks the result before the timeout —
        # both orderings are preserved exactly as authored).
        if now - self._undock_wait_start > self._dock_success_timeout_s:
            self._undock_wait_start = None
            self._enter_error(cmds, now, 'timed out waiting for undock result')
            return
        result = self.pending_undock_result
        if result is not None and self._result_newer(result, self._undock_req_epoch, self._undock_req_seq):
            if result[2]:
                self._resume_task(cmds, now)
            else:
                self._enter_error(cmds, now, 'undock failed')
            return
        self._schedule_timer(cmds, 0.5, 'undock_poll')

    def _resume_task(self, cmds: list[Command], now: float) -> None:
        if self.saved_task is not None:
            self._set_state(cmds, MissionState.RESUMING_TASK, f'returning to paused task {self.saved_task}')
            self._try_send_nav_goal(cmds, self.saved_task, 'task', now)
        else:
            self._set_state(cmds, MissionState.RESUMING_TASK, 'no paused task')
            self._try_send_nav_goal(cmds, self._pre_dock_waypoint, 'resume_none', now)

    # ------------------------------------------------------------ watchdog / timers
    def _on_tf_check_result(self, cmds: list[Command], now: float, ok: bool) -> None:
        # The shell is responsible for skipping this entirely while
        # IDLE/ERROR_WAITING_HUMAN (matching the original _watchdog's early
        # return), so _last_tf_ok_time gets no head start on leaving those
        # states — preserve that no-grace-period behavior by not re-checking
        # state here.
        if ok:
            self._last_tf_ok_time = now
        if self._last_tf_ok_time is None:
            return
        lost = now - self._last_tf_ok_time
        if lost > self._tf_loss_timeout_s:
            self._enter_error(cmds, now, f'localization lost: map->base_link TF missing for {lost:.0f}s')

    def _on_timer_fired(self, cmds: list[Command], now: float, timer_id: int) -> None:
        entry = self._pending_timers.pop(timer_id, None)
        if entry is None:
            return  # stale/unknown timer id, ignore
        kind, payload = entry
        if kind == 'dwell':
            self._on_dwell_fired(cmds, now, timer_id)
        elif kind == 'pre_docking':
            self._begin_docking(cmds, now)
        elif kind == 'nav_retry':
            self._on_nav_retry_fired(cmds, now, payload)
        elif kind == 'dock_poll':
            self._poll_dock_result(cmds, now)
        elif kind == 'undock_poll':
            self._poll_undock_result(cmds, now)
