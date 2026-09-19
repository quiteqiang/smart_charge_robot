"""Unit tests for the pure MissionCore state machine (ID-7).

No rclpy dependency — see conftest.py. Run with: pytest tests/test_mission_core.py -v
"""
from __future__ import annotations

import math

import pytest

from smart_charge_mission.mission_core import MissionCore
from smart_charge_mission.mission_types import (
    BatteryReading,
    CallDockService,
    CancelNavGoal,
    DockResultReceived,
    DockServiceResponse,
    GotoRequested,
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
from smart_charge_mission.mission_types import MissionState as S

WAYPOINTS = {
    'work_1': (15.0, 4.0, 1.5708),
    'work_2': (13.0, 15.0, 3.1416),
    'pre_dock': (3.2, 17.0, 0.0),
}


def make_core(**overrides) -> MissionCore:
    config = dict(
        waypoints=WAYPOINTS,
        frame='map',
        task_waypoints=['work_1', 'work_2'],
        pre_dock_waypoint='pre_dock',
        low_soc_threshold=0.25,
        resume_soc_threshold=0.85,
        max_nav_retries=2,
        max_docking_retries=3,
        dock_success_timeout_s=60.0,
        charge_start_timeout_s=15.0,
        tf_loss_timeout_s=5.0,
    )
    config.update(overrides)
    return MissionCore(**config)


def cmds_of(cmds, cls):
    return [c for c in cmds if isinstance(c, cls)]


def only(cmds, cls):
    matches = cmds_of(cmds, cls)
    assert len(matches) == 1, f'expected exactly one {cls.__name__} in {cmds}'
    return matches[0]


def states_published(cmds):
    return [c.state for c in cmds_of(cmds, PublishState)]


# ------------------------------------------------------------ 1. scaffold


def test_initial_state_is_idle():
    core = make_core()
    assert core.state == S.IDLE
    assert core.soc == 1.0
    assert core.task_queue == []


# ------------------------------------------------------------ 2. simple synchronous transitions


def test_start_task_from_idle_dispatches_first_waypoint_and_acks():
    core = make_core()
    cmds = core.handle(StartTaskRequested(now=0.0))
    assert states_published(cmds) == [S.EXECUTING_TASK]
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'work_1'
    assert goal.source == 'task'
    ack = only(cmds, TriggerAck)
    assert ack.success is True
    assert core.task_queue == ['work_2']


def test_start_task_rejected_when_not_idle():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    cmds = core.handle(StartTaskRequested(now=1.0))
    ack = only(cmds, TriggerAck)
    assert ack.success is False
    assert cmds_of(cmds, SendNavGoal) == []


def test_reset_rejected_outside_error_state():
    core = make_core()
    cmds = core.handle(ResetRequested(now=0.0))
    ack = only(cmds, TriggerAck)
    assert ack.success is False
    assert core.state == S.IDLE


def test_reset_from_error_clears_context_and_returns_to_idle():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    core.handle(NavGoalResponse(now=1.0, request_id=1, accepted=False))
    core.handle(NavGoalResponse(now=1.0, request_id=1, accepted=False))  # exhaust retries (max=2)
    assert core.state == S.ERROR_WAITING_HUMAN

    cmds = core.handle(ResetRequested(now=2.0))
    assert only(cmds, TriggerAck).success is True
    assert states_published(cmds) == [S.IDLE]
    assert core.saved_task is None
    assert core.task_queue == []
    assert core.nav_retries == 0


def test_goto_from_idle_starts_a_single_task():
    core = make_core()
    cmds = core.handle(GotoRequested(now=0.0, waypoint='work_2'))
    assert states_published(cmds) == [S.EXECUTING_TASK]
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'work_2'
    assert core.task_queue == []  # goto bypasses the configured task_waypoints queue


def test_goto_unknown_waypoint_is_ignored():
    core = make_core()
    cmds = core.handle(GotoRequested(now=0.0, waypoint='nope'))
    assert cmds_of(cmds, SendNavGoal) == []
    assert core.state == S.IDLE


def test_goto_while_executing_task_preempts_without_touching_queue():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    assert core.task_queue == ['work_2']

    cmds = core.handle(GotoRequested(now=1.0, waypoint='pre_dock'))
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'pre_dock'
    assert goal.source == 'task'
    # Preemption asymmetry: task_queue and saved_task are untouched.
    assert core.task_queue == ['work_2']
    assert core.saved_task is None


def test_goto_duplicate_target_is_deduped():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))  # active target now 'work_1'
    cmds = core.handle(GotoRequested(now=1.0, waypoint='work_1'))
    assert cmds == []


def test_goto_ignored_from_other_states():
    core = make_core()
    core.handle(BatteryReading(now=0.0, percentage=0.10, charging=False))
    assert core.state == S.LOW_BATTERY
    cmds = core.handle(GotoRequested(now=0.1, waypoint='work_1'))
    assert cmds_of(cmds, SendNavGoal) == []


def test_advance_task_reaches_idle_when_queue_empty():
    core = make_core(task_waypoints=['work_1'])
    core.handle(StartTaskRequested(now=0.0))
    cmds = core.handle(NavResult(now=1.0, request_id=1, success=True))
    assert states_published(cmds) == [S.IDLE]


# ------------------------------------------------------------ 3. nav retry + exhaustion


def test_nav_failure_retries_with_backoff_then_resends():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))  # request_id=1, 'work_1'
    cmds = core.handle(NavResult(now=1.0, request_id=1, success=False))
    assert core.nav_retries == 1
    timer = only(cmds, ScheduleTimer)
    assert timer.delay_s == 1.0

    cmds = core.handle(TimerFired(now=2.0, timer_id=timer.timer_id))
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'work_1'
    assert goal.request_id != 1


def test_nav_retry_exhaustion_enters_error():
    core = make_core()  # max_nav_retries=2
    core.handle(StartTaskRequested(now=0.0))
    core.handle(NavResult(now=1.0, request_id=1, success=False))
    cmds = core.handle(NavResult(now=2.0, request_id=2, success=False))
    assert core.state == S.ERROR_WAITING_HUMAN
    assert states_published(cmds)[-1] == S.ERROR_WAITING_HUMAN


def test_stale_nav_retry_timer_after_state_moved_on_is_a_noop():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    cmds = core.handle(NavResult(now=1.0, request_id=1, success=False))
    timer = only(cmds, ScheduleTimer)
    # Low battery interrupts before the 1s backoff elapses.
    core.handle(BatteryReading(now=1.1, percentage=0.10, charging=False))
    cmds = core.handle(TimerFired(now=2.0, timer_id=timer.timer_id))
    assert cmds_of(cmds, SendNavGoal) == []


def test_rejected_goal_counts_as_a_failure():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    cmds = core.handle(NavGoalResponse(now=1.0, request_id=1, accepted=False))
    assert core.nav_retries == 1
    assert cmds_of(cmds, ScheduleTimer) != []


def test_nav_server_unavailable_goes_straight_to_error_no_retry():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    cmds = core.handle(NavServerUnavailable(now=1.0, request_id=1))
    assert core.state == S.ERROR_WAITING_HUMAN
    assert core.nav_retries == 0  # bypassed the retry counter entirely


# ------------------------------------------------------------ 4. low battery + dwell guard


def test_low_battery_from_idle_has_no_saved_task():
    core = make_core()
    cmds = core.handle(BatteryReading(now=0.0, percentage=0.10, charging=False))
    assert states_published(cmds) == [S.LOW_BATTERY]
    assert core.saved_task is None


def test_low_battery_from_executing_task_saves_task_and_dwells_then_docks():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))  # active target 'work_1'
    cmds = core.handle(BatteryReading(now=1.0, percentage=0.10, charging=False))
    assert states_published(cmds) == [S.LOW_BATTERY]
    assert core.saved_task == 'work_1'
    timer = only(cmds, ScheduleTimer)
    assert timer.delay_s == 1.0

    cmds = core.handle(TimerFired(now=2.0, timer_id=timer.timer_id))
    assert states_published(cmds) == [S.NAVIGATING_TO_DOCK]
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'pre_dock'
    assert goal.source == 'dock'


def test_orphaned_dwell_timer_from_earlier_episode_is_ignored():
    core = make_core()
    core.handle(TfCheckResult(now=-1.0, ok=True))  # baseline so the watchdog has something to lose
    cmds = core.handle(BatteryReading(now=0.0, percentage=0.10, charging=False))
    first_dwell = only(cmds, ScheduleTimer).timer_id

    # Force out of LOW_BATTERY via a TF loss before the first dwell fires,
    # then reset and re-trigger low battery — a second dwell timer is minted.
    cmds = core.handle(TfCheckResult(now=-1.0 + 5.1, ok=False))
    assert core.state == S.ERROR_WAITING_HUMAN
    core.handle(ResetRequested(now=6.1))
    cmds = core.handle(BatteryReading(now=6.2, percentage=0.10, charging=False))
    second_dwell = only(cmds, ScheduleTimer).timer_id
    assert second_dwell != first_dwell

    # The orphaned first dwell timer fires late; must not prematurely advance
    # the second (still-dwelling) episode.
    cmds = core.handle(TimerFired(now=1.0, timer_id=first_dwell))
    assert core.state == S.LOW_BATTERY
    assert cmds == []

    # The real (second) dwell timer still works.
    cmds = core.handle(TimerFired(now=7.2, timer_id=second_dwell))
    assert core.state == S.NAVIGATING_TO_DOCK


def test_unknown_pre_dock_waypoint_enters_error():
    core = make_core(pre_dock_waypoint='nonexistent')
    core.handle(BatteryReading(now=0.0, percentage=0.10, charging=False))
    cmds = core.handle(TimerFired(now=1.0, timer_id=1))
    assert core.state == S.ERROR_WAITING_HUMAN


# ------------------------------------------------------------ 5. docking sequence


def _drive_to_docking(core, now=0.0):
    cmds = core.handle(BatteryReading(now=now, percentage=0.10, charging=False))
    dwell_timer = only(cmds, ScheduleTimer).timer_id
    cmds = core.handle(TimerFired(now=now + 1.0, timer_id=dwell_timer))  # -> NAVIGATING_TO_DOCK
    dock_goal = only(cmds, SendNavGoal)
    cmds = core.handle(NavResult(now=now + 2.0, request_id=dock_goal.request_id, success=True))  # -> PRE_DOCKING
    pre_dock_timer = only(cmds, ScheduleTimer).timer_id
    core.handle(TimerFired(now=now + 2.5, timer_id=pre_dock_timer))  # -> DOCKING
    assert core.state == S.DOCKING


def test_dock_start_success_polls_then_succeeds():
    core = make_core()
    _drive_to_docking(core)
    cmds = core.handle(DockServiceResponse(now=3.0, kind='start', accepted=True))
    poll_timer = only(cmds, ScheduleTimer)

    core.handle(DockResultReceived(now=3.1, sequence=1, success=True))
    cmds = core.handle(TimerFired(now=3.5, timer_id=poll_timer.timer_id))
    assert core.state == S.CHARGING or any(isinstance(c, PublishChargingActive) and c.active for c in cmds)
    assert core.dock_retries == 0


def test_dock_service_unavailable_counts_as_a_failure_and_retries():
    core = make_core()
    _drive_to_docking(core)
    cmds = core.handle(DockServiceResponse(now=3.0, kind='start', accepted=False))
    assert core.dock_retries == 1
    assert states_published(cmds) == [S.NAVIGATING_TO_DOCK]
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'pre_dock'
    assert goal.source == 'dock'


def test_dock_result_stale_replay_is_rejected_by_epoch_seq_gate():
    core = make_core()
    _drive_to_docking(core)
    # A latched replay arrives with sequence <= the request baseline (0) — ignored.
    core.handle(DockResultReceived(now=2.6, sequence=0, success=True))
    cmds = core.handle(DockServiceResponse(now=3.0, kind='start', accepted=True))
    poll_timer = only(cmds, ScheduleTimer)
    cmds = core.handle(TimerFired(now=3.5, timer_id=poll_timer.timer_id))
    # No result newer than baseline yet -> just reschedules the poll.
    assert core.state == S.DOCKING
    assert only(cmds, ScheduleTimer)


def test_dock_result_sequence_rollback_bumps_epoch_and_is_accepted():
    core = make_core()
    core.handle(DockResultReceived(now=0.0, sequence=5, success=False))
    assert core._result_epoch == 0
    core.handle(DockResultReceived(now=0.1, sequence=1, success=True))  # rollback
    assert core._result_epoch == 1
    assert core.pending_dock_result == (1, 1, True)


def test_dock_retry_exhaustion_enters_error():
    core = make_core(max_docking_retries=1)
    _drive_to_docking(core)
    cmds = core.handle(DockServiceResponse(now=3.0, kind='start', accepted=False))
    assert core.state == S.ERROR_WAITING_HUMAN


def test_dock_total_wait_timeout_fails_and_retries():
    core = make_core(dock_success_timeout_s=5.0)
    _drive_to_docking(core)
    cmds = core.handle(DockServiceResponse(now=3.0, kind='start', accepted=True))
    poll_timer = only(cmds, ScheduleTimer)
    cmds = core.handle(TimerFired(now=3.0 + 5.1, timer_id=poll_timer.timer_id))
    assert core.dock_retries == 1
    assert states_published(cmds) == [S.NAVIGATING_TO_DOCK]


# ------------------------------------------------------------ 6. charging / undocking / resume


def _drive_to_charging(core, now=0.0):
    _drive_to_docking(core, now=now)
    cmds = core.handle(DockServiceResponse(now=now + 3.0, kind='start', accepted=True))
    dock_poll_timer = only(cmds, ScheduleTimer).timer_id
    core.handle(DockResultReceived(now=now + 3.1, sequence=1, success=True))
    core.handle(TimerFired(now=now + 3.5, timer_id=dock_poll_timer))
    assert core._charge_request_time is not None
    cmds = core.handle(BatteryReading(now=now + 3.6, percentage=0.30, charging=True))
    assert core.state == S.CHARGING
    return cmds


def test_charge_handshake_timeout_enters_error():
    core = make_core(charge_start_timeout_s=10.0)
    _drive_to_docking(core)
    cmds = core.handle(DockServiceResponse(now=3.0, kind='start', accepted=True))
    dock_poll_timer = only(cmds, ScheduleTimer).timer_id
    core.handle(DockResultReceived(now=3.1, sequence=1, success=True))
    core.handle(TimerFired(now=3.5, timer_id=dock_poll_timer))
    cmds = core.handle(BatteryReading(now=3.5 + 10.1, percentage=0.30, charging=False))
    assert core.state == S.ERROR_WAITING_HUMAN


def test_resume_threshold_crossing_starts_undock():
    core = make_core()
    _drive_to_charging(core)
    cmds = core.handle(BatteryReading(now=100.0, percentage=0.90, charging=True))
    assert states_published(cmds) == [S.UNDOCKING]
    assert only(cmds, CallDockService).kind == 'undock'
    assert only(cmds, ScheduleTimer)  # undock poll starts immediately


def test_undock_success_with_saved_task_resumes_it():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))
    _drive_to_charging(core, now=1.0)
    assert core.saved_task == 'work_1'
    cmds = core.handle(BatteryReading(now=100.0, percentage=0.90, charging=True))
    poll_timer = only(cmds, ScheduleTimer)
    core.handle(DockResultReceived(now=100.1, sequence=2, success=True))
    cmds = core.handle(TimerFired(now=100.5, timer_id=poll_timer.timer_id))
    assert states_published(cmds) == [S.RESUMING_TASK]
    goal = only(cmds, SendNavGoal)
    assert goal.waypoint_name == 'work_1'
    assert goal.source == 'task'

    cmds = core.handle(NavResult(now=110.0, request_id=goal.request_id, success=True))
    assert core.saved_task is None
    assert states_published(cmds) == [S.EXECUTING_TASK]
    assert only(cmds, SendNavGoal).waypoint_name == 'work_2'


def test_undock_success_without_saved_task_goes_idle():
    core = make_core()
    _drive_to_charging(core)  # no task was ever started
    cmds = core.handle(BatteryReading(now=100.0, percentage=0.90, charging=True))
    poll_timer = only(cmds, ScheduleTimer)
    core.handle(DockResultReceived(now=100.1, sequence=2, success=True))
    cmds = core.handle(TimerFired(now=100.5, timer_id=poll_timer.timer_id))
    goal = only(cmds, SendNavGoal)
    assert goal.source == 'resume_none'

    cmds = core.handle(NavResult(now=110.0, request_id=goal.request_id, success=True))
    assert states_published(cmds) == [S.IDLE]


def test_undock_failure_enters_error():
    core = make_core()
    _drive_to_charging(core)
    cmds = core.handle(BatteryReading(now=100.0, percentage=0.90, charging=True))
    poll_timer = only(cmds, ScheduleTimer)
    core.handle(DockResultReceived(now=100.1, sequence=2, success=False))
    cmds = core.handle(TimerFired(now=100.5, timer_id=poll_timer.timer_id))
    assert core.state == S.ERROR_WAITING_HUMAN


def test_undock_rejected_service_response_enters_error_independent_of_poll():
    core = make_core()
    _drive_to_charging(core)
    cmds = core.handle(BatteryReading(now=100.0, percentage=0.90, charging=True))
    assert core.state == S.UNDOCKING
    cmds = core.handle(DockServiceResponse(now=100.05, kind='undock', accepted=False))
    assert core.state == S.ERROR_WAITING_HUMAN


def test_undock_total_wait_timeout_enters_error():
    core = make_core(dock_success_timeout_s=5.0)
    _drive_to_charging(core)
    cmds = core.handle(BatteryReading(now=100.0, percentage=0.90, charging=True))
    poll_timer = only(cmds, ScheduleTimer)
    cmds = core.handle(TimerFired(now=100.0 + 5.1, timer_id=poll_timer.timer_id))
    assert core.state == S.ERROR_WAITING_HUMAN


# ------------------------------------------------------------ 7. watchdog / validity errors


def test_tf_loss_beyond_timeout_enters_error():
    core = make_core(tf_loss_timeout_s=5.0)
    core.handle(StartTaskRequested(now=0.0))
    core.handle(TfCheckResult(now=0.0, ok=True))
    cmds = core.handle(TfCheckResult(now=5.1, ok=False))
    assert core.state == S.ERROR_WAITING_HUMAN


def test_tf_ok_resets_the_loss_timer():
    core = make_core(tf_loss_timeout_s=5.0)
    core.handle(StartTaskRequested(now=0.0))
    core.handle(TfCheckResult(now=0.0, ok=True))
    core.handle(TfCheckResult(now=4.0, ok=True))
    cmds = core.handle(TfCheckResult(now=8.5, ok=False))
    assert core.state == S.EXECUTING_TASK


@pytest.mark.parametrize('bad', [float('nan'), -0.1, 1.1])
def test_invalid_soc_enters_error_unconditionally(bad):
    core = make_core()
    cmds = core.handle(BatteryReading(now=0.0, percentage=bad, charging=False))
    assert core.state == S.ERROR_WAITING_HUMAN


def test_invalid_soc_errors_even_from_idle():
    core = make_core()
    assert core.state == S.IDLE
    core.handle(BatteryReading(now=0.0, percentage=math.nan, charging=False))
    assert core.state == S.ERROR_WAITING_HUMAN


# ------------------------------------------------------------ 8. cancel-race regressions (#6, #4)


def test_issue_6_low_battery_never_cancels_the_preempted_nav_goal():
    """Regression pin: fails if a cancel_goal_async()-equivalent is ever
    reintroduced into the low-battery path (the pre-fix behavior)."""
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))  # request_id=1, 'work_1', task goal in flight
    core.handle(NavGoalResponse(now=0.5, request_id=1, accepted=True))

    cmds = core.handle(BatteryReading(now=1.0, percentage=0.10, charging=False))
    assert cmds_of(cmds, CancelNavGoal) == []
    timer = only(cmds, ScheduleTimer)

    cmds = core.handle(TimerFired(now=2.0, timer_id=timer.timer_id))
    assert cmds_of(cmds, CancelNavGoal) == []
    dock_goal = only(cmds, SendNavGoal)
    assert dock_goal.waypoint_name == 'pre_dock'
    assert dock_goal.request_id != 1

    # The preempted task goal's late result must be a pure no-op (state/nav
    # side effects only — the Log entry documenting the discard is fine).
    cmds = core.handle(NavResult(now=2.1, request_id=1, success=False))
    assert cmds_of(cmds, SendNavGoal) == []
    assert cmds_of(cmds, CancelNavGoal) == []
    assert cmds_of(cmds, PublishState) == []
    assert core.state == S.NAVIGATING_TO_DOCK


@pytest.mark.xfail(
    reason='issue #4: a late nav-goal accept for an already-superseded '
           'request can clobber tracking of the currently active goal',
    strict=True,
)
def test_issue_4_stale_accept_does_not_clobber_the_active_goal():
    core = make_core()
    core.handle(StartTaskRequested(now=0.0))  # request_id=1 ('work_1', task), unaccepted yet
    cmds = core.handle(BatteryReading(now=0.5, percentage=0.10, charging=False))
    dwell_timer_id = only(cmds, ScheduleTimer).timer_id
    cmds = core.handle(TimerFired(now=1.5, timer_id=dwell_timer_id))
    dock_goal = only(cmds, SendNavGoal)  # request_id=2, 'pre_dock', 'dock'

    # The stale accept for request 1 (the preempted task goal) arrives late.
    core.handle(NavGoalResponse(now=1.6, request_id=1, accepted=True))

    # A subsequent error must cancel the real active goal (request 2), not
    # the stale, already-superseded one (request 1).
    core.handle(TfCheckResult(now=1.7, ok=True))
    cmds = core.handle(TfCheckResult(now=1.7 + 5.1, ok=False))
    cancel = only(cmds, CancelNavGoal)
    assert cancel.request_id == dock_goal.request_id
