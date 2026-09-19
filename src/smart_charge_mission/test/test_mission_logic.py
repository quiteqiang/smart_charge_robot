#!/usr/bin/env python3
"""MissionStateMachine 全部转换边的 pytest 单测。

覆盖（docs/improvement_directions.md #4 要求）：正常路径、错误路径、
重试耗尽、reset、结果世代竞态、孤儿 oneshot 防护。无需 rclpy / ROS。
运行：python3 -m pytest src/smart_charge_mission/test -q（或 colcon test）
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from smart_charge_mission.mission_logic import (  # noqa: E402
    CHARGING, DOCKING, ERROR_WAITING_HUMAN, EXECUTING_TASK, IDLE,
    LOW_BATTERY, NAVIGATING_TO_DOCK, PRE_DOCKING, RESUMING_TASK, UNDOCKING,
    MissionStateMachine,
)

WAYPOINTS = {'work_1': {}, 'work_2': {}, 'pre_dock': {}}


@pytest.fixture
def m():
    return MissionStateMachine(
        waypoints=WAYPOINTS,
        task_waypoints=['work_1', 'work_2'],
        pre_dock_waypoint='pre_dock',
        low_soc_threshold=0.25,
        resume_soc_threshold=0.85,
        max_docking_retries=3,
        max_nav_retries=2,
        dock_success_timeout_s=150.0,
        charge_start_timeout_s=15.0,
        tf_loss_timeout_s=5.0,
    )


def kinds(effects):
    return [e.kind for e in effects]


def nav_goals(effects):
    return [e.args for e in effects if e.kind == 'nav_goal']


def oneshots(effects):
    return [e.args for e in effects if e.kind == 'oneshot']


def assert_no_nav(effects):
    assert 'nav_goal' not in kinds(effects)


def assert_stays(m, state, effects):
    """finding-4 修复：两个独立断言，避免 assert_no_nav() and ... 短路。"""
    assert_no_nav(effects)
    assert m.state == state


def start(m):
    ok, msg, fx = m.start_task()
    assert ok and m.state == EXECUTING_TASK
    m.nav_sent('work_1', 'task')   # 壳在 goal 发出后回报
    return fx


def send_nav(m, fx):
    """按壳契约处理 fx_nav_goal：发 goal -> nav_sent。返回 goal 名。"""
    goals = nav_goals(fx)
    assert len(goals) == 1
    m.nav_sent(*goals[0])
    return goals[0]


def arrive_at_dock(m):
    """start -> 低电量 -> 停靠点导航完成 -> PRE_DOCKING。"""
    start(m)
    m.battery(0.20, False, now=0.0)
    m.plan_charge_route(m._low_battery_dwell_id)
    assert m.state == NAVIGATING_TO_DOCK
    m.nav_sent('pre_dock', 'dock')
    m.nav_done(True, 'pre_dock', 'dock')
    assert m.state == PRE_DOCKING


def dock_successfully(m, now=10.0):
    """PRE_DOCKING -> DOCKING -> 泊靠成功 -> 等充电握手。"""
    fx = m.begin_docking(now)
    assert m.state == DOCKING and 'dock_start' in kinds(fx)
    m.dock_start_response(True)
    m.dock_result(1, True)
    fx = m.poll_dock(now + 1.0)
    assert (True,) in [e.args for e in fx if e.kind == 'charging']
    fx = m.battery(0.30, True, now + 2.0)
    assert m.state == CHARGING
    return fx


# ------------------------------------------------------------ 基本任务流
def test_start_task_from_idle(m):
    fx = start(m)
    assert nav_goals(fx) == [('work_1', 'task')]
    assert m.task_queue == ['work_2']


def test_start_task_rejected_in_non_task_states(m):
    start(m)
    m.battery(0.20, False, now=0.0)   # -> LOW_BATTERY（非 IDLE/EXECUTING_TASK）
    ok, msg, fx = m.start_task()
    assert not ok and '无法开始任务' in msg
    assert fx == [] and m.state == LOW_BATTERY


def test_task_completion_advances_and_finishes(m):
    start(m)
    m.nav_sent('work_1', 'task')   # 壳在 goal 发出后回报
    m.nav_done(True, 'work_1', 'task')
    assert m.state == EXECUTING_TASK
    m.nav_sent('work_2', 'task')   # advance 发出新 goal，壳回报
    m.nav_done(True, 'work_2', 'task')
    assert m.state == IDLE


def test_full_task_cycle_goto(m):
    """场景 1 浓缩：IDLE goto -> 执行 -> 完成。"""
    fx = m.goto('work_1')
    assert nav_goals(fx) == [('work_1', 'task')]
    m.nav_sent('work_1', 'task')
    m.nav_done(True, 'work_1', 'task')
    assert m.state == IDLE


# ------------------------------------------------------------ 低电量 / 充电循环
def test_low_battery_saves_task_and_plans_route(m):
    start(m)
    m.nav_sent('work_1', 'task')   # 壳在 goal 发出后回报
    fx = m.battery(0.20, False, now=0.0)
    assert m.state == LOW_BATTERY
    assert m.saved_task == 'work_1'
    shots = oneshots(fx)
    assert shots and shots[0][1] == 'plan_charge_route'
    fx = m.plan_charge_route(shots[0][2][0])
    assert m.state == NAVIGATING_TO_DOCK
    assert nav_goals(fx) == [('pre_dock', 'dock')]
    assert m.nav_retries == 0 and m.dock_retries == 0


def test_low_battery_in_idle_saves_nothing(m):
    fx = m.battery(0.20, False, now=0.0)
    assert m.state == LOW_BATTERY and m.saved_task is None
    m.plan_charge_route(m._low_battery_dwell_id)
    assert m.state == NAVIGATING_TO_DOCK


def test_plan_charge_route_orphan_guard(m):
    """孤儿 oneshot（世代不符 / 状态已变）不得触发导航。"""
    m.battery(0.20, False, now=0.0)
    dwell = m._low_battery_dwell_id
    fx = m.plan_charge_route(dwell + 999)   # 世代不符
    assert_stays(m, LOW_BATTERY, fx)
    m.tf_lost(6.0)                           # 看门狗转入错误态
    fx = m.plan_charge_route(dwell)          # 状态已变
    assert_stays(m, ERROR_WAITING_HUMAN, fx)


def test_arrive_pre_dock_begins_docking(m):
    arrive_at_dock(m)
    fx = m.begin_docking(now=5.0)
    assert m.state == DOCKING
    assert 'dock_start' in kinds(fx)
    fx = m.dock_start_response(True)
    assert [a[1] for a in oneshots(fx)] == ['poll_dock']


def test_dock_success_charging_handshake(m):
    arrive_at_dock(m)
    dock_successfully(m)
    # 握手完成后再次 poll 不重复发 charging
    fx = m.poll_dock(100.0)
    assert 'charging' not in kinds(fx)


def test_charge_start_handshake_timeout(m):
    arrive_at_dock(m)
    m.begin_docking(now=0.0)
    m.dock_start_response(True)
    m.dock_result(1, True)
    m.poll_dock(now=1.0)          # 泊靠成功，等握手
    fx = m.battery(0.30, False, now=1.0 + 15.1)
    assert m.state == ERROR_WAITING_HUMAN
    assert (False,) in [e.args for e in fx if e.kind == 'charging']


def test_resume_saved_task_after_charge(m):
    """场景 6 浓缩：低电量暂停 -> 充电 -> 恢复被暂停航点 -> 续跑队列。"""
    start(m)
    m.nav_sent('work_1', 'task')
    m.battery(0.20, False, now=0.0)
    m.plan_charge_route(m._low_battery_dwell_id)
    m.nav_sent('pre_dock', 'dock')   # 壳在 goal 发出后回报
    m.nav_done(True, 'pre_dock', 'dock')
    dock_successfully(m)
    fx = m.battery(0.90, True, now=20.0)
    assert m.state == UNDOCKING
    assert 'undock' in kinds(fx)
    m.undock_response(True)
    m.dock_result(2, True)
    fx = m.poll_undock(now=21.0)
    assert m.state == RESUMING_TASK
    assert nav_goals(fx) == [('work_1', 'task')]
    m.nav_sent('work_1', 'task')   # 壳在 goal 发出后回报
    m.nav_done(True, 'work_1', 'task')
    assert m.saved_task is None
    # 剩余队列续跑
    assert m.state == EXECUTING_TASK


def test_resume_none_when_no_saved_task(m):
    """低电量发生在 IDLE：充电完成后直接回 IDLE（resume_none 路径）。"""
    m.battery(0.20, False, now=0.0)
    m.plan_charge_route(m._low_battery_dwell_id)
    m.nav_sent('pre_dock', 'dock')   # 壳在 goal 发出后回报
    m.nav_done(True, 'pre_dock', 'dock')
    dock_successfully(m)
    m.battery(0.90, True, now=20.0)
    m.undock_response(True)
    m.dock_result(2, True)
    fx = m.poll_undock(now=21.0)
    assert m.state == RESUMING_TASK
    assert nav_goals(fx) == [('pre_dock', 'resume_none')]
    m.nav_sent('pre_dock', 'resume_none')   # 壳在 goal 发出后回报
    m.nav_done(True, 'pre_dock', 'resume_none')
    assert m.state == IDLE


def test_soc_below_resume_keeps_charging(m):
    arrive_at_dock(m)
    dock_successfully(m)
    fx = m.battery(0.50, True, now=20.0)
    assert m.state == CHARGING and fx == []


# ------------------------------------------------------------ 导航失败/重试
def test_nav_failure_retries_then_error(m):
    start(m)
    m.nav_sent('work_1', 'task')
    fx = m.nav_done(False, 'work_1', 'task')
    assert m.nav_retries == 1
    assert [a[1] for a in oneshots(fx)] == ['retry_nav']
    m.retry_nav('work_1', 'task')
    m.nav_sent('work_1', 'task')
    m.nav_done(False, 'work_1', 'task')
    assert m.state == ERROR_WAITING_HUMAN


def test_retry_nav_guarded_by_state(m):
    """重试 oneshot 到点时状态已变（低电量转充电），不得再发导航。"""
    start(m)
    m.nav_sent('work_1', 'task')
    m.nav_done(False, 'work_1', 'task')     # 安排 retry oneshot
    m.battery(0.20, False, now=1.0)         # 状态 -> LOW_BATTERY
    fx = m.retry_nav('work_1', 'task')
    assert_no_nav(fx)


def test_late_nav_result_ignored(m):
    """低电量后迟到的任务导航结果（source=task, 状态不符）不得触发重试。"""
    start(m)
    m.nav_sent('work_1', 'task')
    m.battery(0.20, False, now=0.0)   # 保存 work_1，转充电路线
    fx = m.nav_done(False, 'work_1', 'task')
    assert fx == [] and m.nav_retries == 0


def test_nav_unavailable_during_charge_route(m):
    m.battery(0.20, False, now=0.0)
    m.plan_charge_route(m._low_battery_dwell_id)
    fx = m.nav_unavailable('pre_dock', 'dock')
    assert m.state == ERROR_WAITING_HUMAN
    assert fx and fx[-1].kind == 'state'


# ------------------------------------------------------------ 泊靠失败/重试
def test_dock_failure_retries_exhausted(m):
    arrive_at_dock(m)
    for attempt in range(1, 4):
        m.begin_docking(now=float(attempt))
        assert m.state == DOCKING
        m.dock_start_response(True)
        m.dock_result(attempt, False)
        fx = m.poll_dock(now=attempt + 0.5)
        if attempt < 3:
            assert m.state == NAVIGATING_TO_DOCK, f'第 {attempt} 次失败后应退回重试'
            assert m.dock_retries == attempt
            assert nav_goals(fx) == [('pre_dock', 'dock')]
            m.nav_sent('pre_dock', 'dock')   # 壳在 goal 发出后回报
            m.nav_done(True, 'pre_dock', 'dock')
        else:
            assert m.state == ERROR_WAITING_HUMAN
            assert m.dock_retries == 3


def test_dock_start_service_rejected(m):
    arrive_at_dock(m)
    m.begin_docking(now=1.0)
    fx = m.dock_start_response(False, '泊靠服务拒绝启动')
    assert m.state == NAVIGATING_TO_DOCK and m.dock_retries == 1
    assert (False,) in [e.args for e in fx if e.kind == 'charging']


def test_dock_result_wait_timeout(m):
    arrive_at_dock(m)
    m.begin_docking(now=0.0)
    m.dock_start_response(True)
    fx = m.poll_dock(now=1.0)
    assert [a[1] for a in oneshots(fx)] == ['poll_dock']   # 无结果，自续
    fx = m.poll_dock(now=151.0)
    # 超时 = 一次泊靠失败：第 1/3 次，退回预停靠点重试
    assert m.state == NAVIGATING_TO_DOCK and m.dock_retries == 1
    assert nav_goals(fx) == [('pre_dock', 'dock')]


# ------------------------------------------------------------ 结果世代竞态
def test_stale_latched_result_ignored(m):
    """请求基线之后的同世代低序号结果（latched replay）必须忽略。"""
    arrive_at_dock(m)
    m.begin_docking(now=0.0)      # 基线 = 当前世代 + 最近序号(0, latched seq=0)
    m.dock_start_response(True)
    m.dock_result(0, False)       # 旧基线值 replay
    fx = m.poll_dock(now=1.0)
    assert m.state == DOCKING     # 未误判失败
    assert_no_nav(fx)
    m.dock_result(1, True)        # 真实结果
    m.poll_dock(now=1.5)
    assert m._charge_request_time is not None   # 进入握手等待


def test_epoch_bump_on_sequence_rollback(m):
    """控制器重启（序号回退）→ 世代递增，否则真实结果会被旧基线永久拒绝。"""
    m.dock_result(5, True)        # 旧世代结果，最后序号 5
    arrive_at_dock(m)
    m.begin_docking(now=0.0)      # 基线 = (epoch0, seq5)
    m.dock_start_response(True)
    m.dock_result(2, True)        # 重启后新控制器从低序号开始 → epoch1
    assert m._result_epoch == 1
    fx = m.poll_dock(now=1.0)
    assert m._charge_request_time is not None   # 跨世代结果被接受


# ------------------------------------------------------------ 离桩
def test_undock_failure_enters_error(m):
    arrive_at_dock(m)
    dock_successfully(m)
    m.battery(0.90, True, now=20.0)
    m.undock_response(True)
    m.dock_result(2, False)
    m.poll_undock(now=21.0)
    assert m.state == ERROR_WAITING_HUMAN


def test_undock_rejected_enters_error(m):
    arrive_at_dock(m)
    dock_successfully(m)
    m.battery(0.90, True, now=20.0)
    fx = m.undock_response(False, '离桩被拒绝')
    assert m.state == ERROR_WAITING_HUMAN
    assert fx and fx[-1].kind == 'state'


def test_undock_wait_timeout(m):
    arrive_at_dock(m)
    dock_successfully(m)
    m.battery(0.90, True, now=20.0)
    m.undock_response(True)
    fx = m.poll_undock(now=20.0 + 151.0)
    assert m.state == ERROR_WAITING_HUMAN


# ------------------------------------------------------------ 错误/复位/看门狗
def test_invalid_soc_enters_error(m):
    m.battery(math.nan, False, now=0.0)
    assert m.state == ERROR_WAITING_HUMAN
    m2 = MissionStateMachine(waypoints=WAYPOINTS, task_waypoints=[], pre_dock_waypoint='pre_dock',
                             low_soc_threshold=0.25, resume_soc_threshold=0.85,
                             max_docking_retries=3, max_nav_retries=2,
                             dock_success_timeout_s=150.0, charge_start_timeout_s=15.0,
                             tf_loss_timeout_s=5.0)
    m2.battery(1.5, False, now=0.0)
    assert m2.state == ERROR_WAITING_HUMAN


def test_reset_only_from_error(m):
    ok, _, _ = m.reset()
    assert not ok and m.state == IDLE
    m.battery(math.nan, False, now=0.0)
    ok, msg, fx = m.reset()
    assert ok and m.state == IDLE
    assert fx and fx[-1].kind == 'state'


def test_reset_clears_saved_queue_retries(m):
    start(m)
    m.nav_sent('work_1', 'task')
    m.battery(0.20, False, now=0.0)
    m.plan_charge_route(m._low_battery_dwell_id)
    m.nav_sent('pre_dock', 'dock')   # 壳在 goal 发出后回报
    m.nav_done(False, 'pre_dock', 'dock')
    m.retry_nav('pre_dock', 'dock')
    m.nav_sent('pre_dock', 'dock')   # 壳在 goal 发出后回报
    m.nav_done(False, 'pre_dock', 'dock')   # nav_retries=2 -> error
    assert m.state == ERROR_WAITING_HUMAN
    m.reset()
    assert m.saved_task is None and m.task_queue == []
    assert m.nav_retries == 0 and m.dock_retries == 0


def test_error_emits_cancel_nav_and_stop_charging(m):
    start(m)
    fx = m.battery(math.nan, False, now=0.0)
    assert 'cancel_nav' in kinds(fx)
    assert (False,) in [e.args for e in fx if e.kind == 'charging']
    assert fx[-1].args[0] == ERROR_WAITING_HUMAN


def test_tf_lost_enters_error(m):
    start(m)
    fx = m.tf_lost(6.0)
    assert m.state == ERROR_WAITING_HUMAN
    assert '定位失效' in fx[-1].args[1]


# ------------------------------------------------------------ goto 语义
def test_goto_preempts_executing_task(m):
    start(m)
    m.nav_sent('work_1', 'task')
    fx = m.goto('work_2')
    assert nav_goals(fx) == [('work_2', 'task')]


def test_goto_duplicate_target_ignored(m):
    start(m)
    m.nav_sent('work_1', 'task')
    fx = m.goto('work_1')
    assert_no_nav(fx)


def test_goto_ignored_in_charge_cycle(m):
    m.battery(0.20, False, now=0.0)
    fx = m.goto('work_1')
    assert_stays(m, LOW_BATTERY, fx)


def test_goto_unknown_ignored(m):
    fx = m.goto('nowhere')
    assert fx == [] and m.state == IDLE


# ------------------------------------------------------------ 抢占语义 (#7)
def test_goto_preempt_emits_cancel(m):
    """抢占必须显式取消旧目标：壳不 cancel 时旧 goal 的迟到结果会毒化新目标。"""
    start(m)   # active = work_1（start helper 已按壳契约回报 nav_sent）
    fx = m.goto('work_2')
    assert 'cancel_nav' in kinds(fx)
    assert nav_goals(fx) == [('work_2', 'task')]
    m.nav_sent('work_2', 'task')
    assert m._active_nav_target == 'work_2'


def test_stale_result_after_preempt_ignored(m):
    """goto 抢占后，被取消旧目标的迟到失败结果不得触发重试/进错误态。"""
    start(m)
    m.goto('work_2')
    m.nav_sent('work_2', 'task')
    fx = m.nav_done(False, 'work_1', 'task')   # 旧目标被取消的迟到结果
    assert fx == [] and m.state == EXECUTING_TASK
    assert m.nav_retries == 0
    assert m._active_nav_target == 'work_2'   # 新目标不受影响


def test_low_battery_after_preempt_saves_goto_target(m):
    """抢占路径下 saved_task 必须指向用户最后意图的航点（#7 核心保证）。"""
    start(m)
    m.goto('work_2')
    m.nav_sent('work_2', 'task')
    m.battery(0.20, False, now=0.0)
    assert m.saved_task == 'work_2'
    assert m.state == LOW_BATTERY


def test_start_task_preempts_executing(m):
    """start_task 在 EXECUTING_TASK 中也可抢占（修复服务/话题权限倒挂）。"""
    start(m)
    m.nav_done(True, 'work_1', 'task')   # 完成 work_1，advance 发 work_2
    m.nav_sent('work_2', 'task')
    ok, msg, fx = m.start_task()
    assert ok
    assert 'cancel_nav' in kinds(fx)
    assert nav_goals(fx) == [('work_1', 'task')]   # 队列从头重来
    assert m.task_queue == ['work_2']
    assert m.state == EXECUTING_TASK


def test_stale_retry_after_preempt_ignored(m):
    """抢占后 1s 前失败的旧目标重试到点：不得重发已取消目标（review finding 1）。

    时序：work_2 失败一次（nav_retries=1，retry oneshot 挂起）-> start_task
    抢占重来 -> 孤儿 retry(work_2) 到点。无守卫时它会覆盖新目标 work_1，
    再失败两次就把新任务拖进 ERROR_WAITING_HUMAN。
    """
    start(m)
    m.nav_done(True, 'work_1', 'task')
    m.nav_sent('work_2', 'task')
    fx = m.nav_done(False, 'work_2', 'task')   # work_2 失败，安排 1s 后重试
    assert 'oneshot' in kinds(fx) and m.nav_retries == 1
    ok, _, fx = m.start_task()                  # 抢占：取消 work_2，重排队列
    assert ok
    m.nav_sent('work_1', 'task')
    assert m._active_nav_target == 'work_1'
    fx = m.retry_nav('work_2', 'task')          # 孤儿重试到点
    assert fx == []                             # 被守卫丢弃
    assert m._active_nav_target == 'work_1'     # 新目标未被覆盖
    assert m.nav_retries == 0                   # 抢占重置了重试预算


def test_start_task_rejected_in_charge_cycle(m):
    """非 IDLE/EXECUTING_TASK（充电循环中）仍拒绝，抢占不破坏安全状态。"""
    arrive_at_dock(m)
    dock_successfully(m)
    ok, msg, fx = m.start_task()
    assert not ok and '无法开始任务' in msg
    assert fx == [] and m.state == CHARGING


# ------------------------------------------------------------ 端到端浓缩
def test_full_low_battery_cycle(m):
    """场景 3/4/5/6 浓缩：任务中低电量 -> 泊靠 -> 充电 -> 恢复 -> 任务完成。"""
    start(m)
    m.nav_sent('work_1', 'task')
    m.nav_done(True, 'work_1', 'task')          # work_1 完成，执行 work_2
    m.nav_sent('work_2', 'task')
    m.battery(0.20, False, now=1.0)             # 暂停 work_2
    assert m.saved_task == 'work_2'
    m.plan_charge_route(m._low_battery_dwell_id)
    m.nav_sent('pre_dock', 'dock')   # 壳在 goal 发出后回报
    m.nav_done(True, 'pre_dock', 'dock')
    m.begin_docking(now=2.0)
    m.dock_start_response(True)
    m.dock_result(1, True)
    m.poll_dock(now=3.0)
    m.battery(0.30, True, now=4.0)              # 握手成功
    assert m.state == CHARGING
    m.battery(0.90, True, now=30.0)             # 充到恢复阈值
    assert m.state == UNDOCKING
    m.undock_response(True)
    m.dock_result(2, True)
    m.poll_undock(now=31.0)
    assert m.state == RESUMING_TASK
    m.nav_sent('work_2', 'task')   # 壳在 goal 发出后回报
    m.nav_done(True, 'work_2', 'task')          # 恢复并完成最后一个任务
    assert m.state == IDLE


# ------------------------------------------------------------ code-review 修复回归测试
def test_late_dock_start_failure_in_error_ignored(m):
    """/dock/start 失败响应在途时状态机已进错误态：迟到响应不得拖出错误态
    （review finding 2：原实现会从人工监督态把机器人开回充电桩）。"""
    arrive_at_dock(m)
    m.begin_docking(now=0.0)          # dock_start 调用在途（未调 dock_start_response）
    m.tf_lost(6.0)                    # 看门狗先转入错误态
    fx = m.dock_start_response(False, '泊靠服务调用异常: boom')
    assert m.state == ERROR_WAITING_HUMAN
    assert_stays(m, ERROR_WAITING_HUMAN, fx)
    assert m.dock_retries == 0        # 未计入重试


def test_late_undock_failure_after_reset_ignored(m):
    """人工复位后迟到的离桩失败响应不得把机器从 IDLE 弹回错误态。"""
    arrive_at_dock(m)
    dock_successfully(m)
    m.battery(0.90, True, now=20.0)   # UNDOCKING，undock 调用在途
    m.undock_response(True)
    m.dock_result(2, False)           # 离桩失败 -> 错误态
    m.poll_undock(now=21.0)
    assert m.state == ERROR_WAITING_HUMAN
    m.reset()
    assert m.state == IDLE
    # 复位后 controller 侧迟到的重试/失败消息到达
    fx = m.undock_response(False, '离桩被拒绝')
    assert_stays(m, IDLE, fx)


def test_update_config_applies_live(m):
    """ros2 param set 路径（review finding 1）：运行时改配置下次决策生效。"""
    m.update_config(resume_soc_threshold=0.50, max_nav_retries=5)
    assert m.resume_soc_threshold == 0.50 and m.max_nav_retries == 5
    arrive_at_dock(m)
    dock_successfully(m)
    m.battery(0.60, True, now=20.0)   # 0.60 >= 新的 0.50 阈值
    assert m.state == UNDOCKING


def test_shell_logger_dispatch_has_fixed_severity_per_line():
    """防回归（集成测试曾抓到的崩溃）：rclpy 按调用点缓存 severity，
    同一源码行用 getattr 等动态分发不同级别会抛
    ValueError('Logger severity cannot be changed between calls') 并杀死节点。
    壳的 _log 必须为每个级别使用固定的独立源码行。"""
    shell = os.path.join(os.path.dirname(__file__), '..',
                         'smart_charge_mission', 'charge_mission_node.py')
    src = open(shell, encoding='utf-8').read()
    assert 'getattr(self.get_logger()' not in src
    assert 'getattr(logger' not in src
