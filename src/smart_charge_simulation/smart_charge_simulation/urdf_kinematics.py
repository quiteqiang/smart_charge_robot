#!/usr/bin/env python3
"""从 URDF 解析运动学安装参数（纯 Python，可 pytest 秒级覆盖）。

对应 docs/improvement_directions.md #8：wheel_radius / wheel_separation /
laser_x 此前在 URDF 与 sim_params.yaml 两处手工同步，改一忘二就会让
robot_state_publisher 发布的 TF 与仿真真值漂移。URDF 是单一事实源，
本模块负责抽取；yaml 参数降级为 URDF 缺失/不可解析时的 fallback。

抽取规则（刻意简单、对本项目 URDF 充分）：
  - laser_x：child link 为 base_laser 的 fixed joint 的 origin x；
  - 轮半径：名字含 "wheel" 的 link 的 cylinder 几何半径；
  - 轮距：左右轮 joint origin y 之差。
任一项缺失返回 None（调用方回退 yaml 参数）。
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass(frozen=True)
class Kinematics:
    wheel_radius: float      # m
    wheel_separation: float  # m（左右轮 joint y 之差）
    laser_x: float           # base_laser 在 base_link 中的安装位置 x (m)


def _origin_xyz(joint: ET.Element) -> tuple[float, float, float]:
    origin = joint.find('origin')
    if origin is None or not origin.get('xyz'):
        return 0.0, 0.0, 0.0
    x, y, z = origin.get('xyz').split()[:3]
    return float(x), float(y), float(z)


def parse_kinematics(urdf_xml: str) -> Kinematics | None:
    """解析 URDF 文本；缺项/不可解析返回 None（不抛异常）。"""
    # 承诺不抛异常：属性缺失/非数值等"可解析但畸形"的 URDF 一律回退 yaml，
    # 不能让 sim 节点死在启动路径上（review finding 3）
    try:
        root = ET.fromstring(urdf_xml)
        laser_x: float | None = None
        wheels: list[tuple[float, float]] = []   # (joint origin y, wheel radius)

        for joint in root.iter('joint'):
            child = joint.find('child')
            if child is None:
                continue
            child_name = child.get('link', '')
            _, y, _ = _origin_xyz(joint)
            if child_name == 'base_laser':
                x, _, _ = _origin_xyz(joint)
                laser_x = x
            elif 'wheel' in child_name:
                link = root.find(f".//link[@name='{child_name}']")
                cyl = link.find('.//geometry/cylinder') if link is not None else None
                if cyl is not None and cyl.get('radius'):
                    wheels.append((y, float(cyl.get('radius'))))

        if laser_x is None or len(wheels) < 2:
            return None
        (y1, r1), (y2, r2) = wheels[0], wheels[1]
        return Kinematics(wheel_radius=(r1 + r2) / 2.0,
                          wheel_separation=abs(y1 - y2),
                          laser_x=laser_x)
    except (ValueError, ET.ParseError):
        return None
