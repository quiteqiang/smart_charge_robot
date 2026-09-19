#!/usr/bin/env python3
"""BatteryModel 行为模型的 pytest 单测。

覆盖 docs/improvement_directions.md #6 的三项改进：
CC/CV 充电曲线、电压一阶滞后 + 负载压降、Ah 电荷记账（SOC=charge/capacity），
以及握手等待、放电速度插值、set_soc 注入。无需 rclpy / ROS。
运行：python3 -m pytest src/smart_charge_battery/test -q
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from smart_charge_battery.battery_model import BatteryModel  # noqa: E402


def make(**kw):
    params = dict(capacity_ah=100.0, initial_soc=0.5)
    params.update(kw)
    return BatteryModel(**params)


# ---------------------------------------------------------------- CC/CV 充电
def test_cc_phase_full_rate_below_threshold():
    m = make(initial_soc=0.5)
    snap = m.step(1.0, docked=True, charge_requested=True, charge_rate=0.01)
    assert snap.charging
    assert snap.soc == pytest.approx(0.5 + 0.01)      # CC 段满速
    assert snap.current == pytest.approx(-0.01 * 100.0)  # -1 A，负 = 充入


def test_cv_phase_tapers_linearly():
    m = make(initial_soc=0.9)   # threshold 0.8 → taper = (1-0.9)/0.2 = 0.5
    snap = m.step(1.0, docked=True, charge_requested=True, charge_rate=0.01)
    assert snap.soc == pytest.approx(0.9 + 0.01 * 0.5)


def test_cv_phase_rate_approaches_zero_at_full():
    m = make(initial_soc=0.99)
    snap = m.step(1.0, docked=True, charge_requested=True, charge_rate=0.01)
    # taper = 0.05：仍在充但极慢，且不超过 1.0
    assert snap.soc == pytest.approx(0.99 + 0.01 * 0.05)
    m2 = make(initial_soc=1.0)
    snap2 = m2.step(1.0, docked=True, charge_requested=True, charge_rate=0.01)
    assert not snap2.charging          # soc < 1.0 是充电前提
    assert snap2.soc == 1.0            # 满电挂桩保持，不掉入放电分支
    assert snap2.current == 0.0


def test_charge_taper_helper():
    m = make()
    assert m.charge_taper(0.0) == 1.0
    assert m.charge_taper(0.8) == pytest.approx(1.0)
    assert m.charge_taper(0.9) == pytest.approx(0.5)
    assert m.charge_taper(1.0) == 0.0


# ------------------------------------------------------------------ 握手/放电
def test_docked_without_request_holds_charge():
    m = make(initial_soc=0.5)
    snap = m.step(10.0, docked=True, charge_requested=False)
    assert snap.soc == pytest.approx(0.5)   # 等待握手，电量不变
    assert snap.current == 0.0


def test_idle_discharge_uses_idle_rate():
    m = make(initial_soc=0.5)
    snap = m.step(1.0, speed=0.0)
    assert snap.soc == pytest.approx(0.5 - 0.0001)


def test_full_speed_discharge_uses_discharge_rate():
    m = make(initial_soc=0.5, max_speed=0.5)
    snap = m.step(1.0, speed=0.5)
    assert snap.soc == pytest.approx(0.5 - 0.002)
    assert snap.current == pytest.approx(0.002 * 100.0)


def test_half_speed_interpolates_rate():
    m = make(initial_soc=0.5, max_speed=0.5)
    snap = m.step(1.0, speed=0.25)
    expected = 0.0001 + (0.002 - 0.0001) * 0.5
    assert snap.soc == pytest.approx(0.5 - expected)


def test_soc_clamped_at_zero():
    m = make(initial_soc=0.001)
    snap = m.step(10.0, speed=1.0)
    assert snap.soc == 0.0


# ------------------------------------------------------------------ 电压动态
def test_voltage_initialized_to_ocv():
    m = make(initial_soc=0.5)
    assert m.voltage == pytest.approx(24.0)   # 22 + 4*0.5


def test_voltage_sags_under_load_and_recovers():
    m = make(initial_soc=0.5, internal_resistance=2.0, voltage_tau_s=0.5)
    # 满速放电电流 0.2 A；dt=0.5 后 soc=0.499 → v_target = OCV(0.499) - 0.4
    snap = m.step(0.5, speed=0.5)
    alpha = 1.0 - math.exp(-0.5 / 0.5)
    v_target = (22.0 + 4.0 * 0.499) - 0.002 * 100.0 * 2.0
    assert snap.voltage == pytest.approx(24.0 + (v_target - 24.0) * alpha)
    assert snap.voltage < 24.0
    # 静置后电压逐渐回升逼近 OCV(0.498) - idle 压降
    for _ in range(20):
        snap = m.step(0.5, speed=0.0)
    assert snap.voltage == pytest.approx((22.0 + 4.0 * 0.498) - 0.0001 * 100.0 * 2.0,
                                         abs=1e-3)


def test_voltage_tau_zero_disables_lag():
    m = make(initial_soc=0.5, internal_resistance=2.0, voltage_tau_s=0.0)
    snap = m.step(0.5, speed=0.5)
    # 立即贴合目标（仍用推进后的 soc=0.499）
    assert snap.voltage == pytest.approx((22.0 + 4.0 * 0.499) - 0.002 * 100.0 * 2.0)


# ------------------------------------------------------------------ 记账/注入
def test_bookkeeping_matches_capacity():
    m = make(capacity_ah=50.0, initial_soc=0.5)
    snap = m.step(1.0, docked=True, charge_requested=True, charge_rate=0.01)
    # 50 Ah 容量：0.01 SOC/s = 0.5 A
    assert snap.current == pytest.approx(-0.5)
    assert snap.soc == pytest.approx(0.51)


def test_set_soc_resets_charge_and_voltage():
    m = make(initial_soc=0.2)
    m.set_soc(0.8)
    assert m.soc == pytest.approx(0.8)
    assert m.voltage == pytest.approx(22.0 + 4.0 * 0.8)


def test_set_soc_clamps_out_of_range():
    m = make(initial_soc=0.5)
    m.set_soc(1.7)
    assert m.soc == 1.0
    m.set_soc(-0.3)
    assert m.soc == 0.0


def test_dt_negative_treated_as_zero():
    m = make(initial_soc=0.5)
    snap = m.step(-1.0, speed=0.5)
    assert snap.soc == pytest.approx(0.5)
