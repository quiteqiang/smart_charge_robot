#!/usr/bin/env python3
"""纯 Python 电池模型（不依赖 rclpy，可 pytest 秒级覆盖）。

对应 docs/improvement_directions.md #6，三项改进：
  1. CC/CV 两段充电：SOC < cc_cv_threshold 时满速（恒流段），之后线性
     降速到 SOC=1 时的 0（恒压涓流段）——"充到 85% 要多久"更接近真实；
  2. 电压一阶滞后 + 负载压降：v_target = OCV(soc) - |I|·R，端电压以一阶
     惯性逼近目标值，/battery_state.voltage 不再与 SOC 完全线性相关；
  3. 真实 bookkeeping：内部以电荷量 (Ah) 记账，SOC = charge / capacity，
     percentage 与 capacity 天然耦合。

节点壳 battery_simulator_node.py 只做 ROS IO；本模块全部行为可用
test/test_battery_model.py 单测覆盖。电流约定：正 = 放电，负 = 充电 (A)，
与 sensor_msgs/BatteryState.current 一致。

注：仿真时间尺度被压缩（满放约 500s，等效 ~50C），内阻默认值取 2 Ω
使压降量级可见，并非真实电芯内阻；接真实 BMS 时整模型替换即可。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class BatterySnapshot:
    """一次 step 后的电池状态快照（壳据此发布 /battery_state）。"""
    soc: float
    voltage: float
    current: float     # 正 = 放电，负 = 充电 (A)，= -d(charge_ah)/dt
    charging: bool     # 处于充电状态（docked & charge_requested & soc < 1）


class BatteryModel:
    """SOC/电压演化模型。速率参数由 step() 传入，支持运行时调参。"""

    def __init__(self, *, capacity_ah: float = 100.0, initial_soc: float = 1.0,
                 max_speed: float = 0.5, cc_cv_threshold: float = 0.8,
                 internal_resistance: float = 2.0,
                 voltage_tau_s: float = 2.0) -> None:
        self.capacity_ah = float(capacity_ah)
        if self.capacity_ah <= 0:
            raise ValueError(
                f'capacity_ah 必须为正数，收到 {capacity_ah}')
        self.max_speed = float(max_speed)
        self.cc_cv_threshold = float(cc_cv_threshold)
        self.internal_resistance = float(internal_resistance)
        self.voltage_tau_s = float(voltage_tau_s)
        self._charge_ah = self._clamp01(initial_soc) * self.capacity_ah
        self._voltage = self._ocv(self.soc)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _clamp01(x: float) -> float:
        return min(1.0, max(0.0, float(x)))

    @staticmethod
    def _ocv(soc: float) -> float:
        """开路电压 22V(空) ~ 26V(满)。"""
        return 22.0 + 4.0 * soc

    @property
    def soc(self) -> float:
        return self._clamp01(self._charge_ah / self.capacity_ah)

    @property
    def voltage(self) -> float:
        return self._voltage

    def charge_taper(self, soc: float) -> float:
        """CC/CV 降额系数：CC 段（soc < 阈值）为 1.0；CV 段线性降到 0。"""
        if soc < self.cc_cv_threshold:
            return 1.0
        span = max(1.0 - self.cc_cv_threshold, 1e-6)
        return max(0.0, (1.0 - soc) / span)

    def set_soc(self, soc: float) -> None:
        """测试注入：重置电荷量，端电压立即贴合 OCV。"""
        self._charge_ah = self._clamp01(soc) * self.capacity_ah
        self._voltage = self._ocv(self.soc)

    # ------------------------------------------------------------------ #
    def step(self, dt: float, *, speed: float = 0.0, docked: bool = False,
             charge_requested: bool = False, charge_rate: float = 0.01,
             discharge_rate: float = 0.002,
             idle_discharge_rate: float = 0.0001) -> BatterySnapshot:
        """推进一个周期。dt 秒；返回推进后的快照。"""
        dt = max(0.0, float(dt))
        soc = self.soc
        charging = bool(docked) and bool(charge_requested) and soc < 1.0

        if docked and (not charging):
            # 已连接但未拉起充电请求（等待握手），或已充满：均保持电量，
            # 不掉入放电分支（否则满电挂桩会 idle 放电再回充，出现振荡）
            current = 0.0
        elif charging:
            # CC/CV：有效速率 = 满速 × 降额系数；CV 段逼近 SOC=1 时速率→0
            effective_rate = charge_rate * self.charge_taper(soc)   # SOC/s
            current = -effective_rate * self.capacity_ah            # A
            self._charge_ah = min(self.capacity_ah,
                                  self._charge_ah
                                  + effective_rate * self.capacity_ah * dt)
        else:
            # 放电：行驶速率按 |v|/max_speed 在 idle 与 discharge 间插值
            scale = (min(max(speed, 0.0) / self.max_speed, 1.0)
                     if self.max_speed > 0 else 0.0)
            rate = idle_discharge_rate + (discharge_rate - idle_discharge_rate) * scale
            current = rate * self.capacity_ah
            self._charge_ah = max(0.0, self._charge_ah
                                  - rate * self.capacity_ah * dt)

        # 端电压：一阶滞后逼近 OCV - |I|·R（充放电均下垂）
        soc = self.soc
        v_target = self._ocv(soc) - abs(current) * self.internal_resistance
        if self.voltage_tau_s > 0:
            alpha = 1.0 - math.exp(-dt / self.voltage_tau_s)
            self._voltage += (v_target - self._voltage) * alpha
        else:
            self._voltage = v_target
        return BatterySnapshot(soc=soc, voltage=self._voltage,
                               current=current, charging=charging)
