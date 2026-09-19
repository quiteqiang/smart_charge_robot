#!/usr/bin/env python3
"""URDF 运动学参数抽取的 pytest 单测（improvement_directions #8）。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from smart_charge_simulation.urdf_kinematics import parse_kinematics  # noqa: E402

# 与 src/smart_charge_base/urdf/mining_truck.urdf 结构一致的最小片段
MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="test_truck">
  <link name="base_link"/>
  <link name="base_laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/><child link="base_laser"/>
    <origin xyz="0.15 0.0 0.38"/>
  </joint>
  <link name="wheel_left">
    <visual><geometry><cylinder radius="0.10" length="0.04"/></geometry></visual>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_left"/>
    <origin xyz="0.0 0.21 0.10"/><axis xyz="0 1 0"/>
  </joint>
  <link name="wheel_right">
    <visual><geometry><cylinder radius="0.10" length="0.04"/></geometry></visual>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_right"/>
    <origin xyz="0.0 -0.21 0.10"/><axis xyz="0 1 0"/>
  </joint>
</robot>
"""


def test_parse_minimal_urdf():
    kin = parse_kinematics(MINIMAL_URDF)
    assert kin is not None
    assert kin.laser_x == 0.15
    assert kin.wheel_radius == 0.10
    assert kin.wheel_separation == 0.42


def test_parse_real_project_urdf():
    urdf = os.path.join(os.path.dirname(__file__), '..', '..',
                        'smart_charge_base', 'urdf', 'mining_truck.urdf')
    kin = parse_kinematics(open(urdf, encoding='utf-8').read())
    assert kin is not None
    assert kin.laser_x == 0.15
    assert kin.wheel_radius == 0.10
    assert kin.wheel_separation == 0.42


def test_missing_wheels_returns_none():
    no_wheels = MINIMAL_URDF.replace('wheel_left', 'tire_left') \
                            .replace('wheel_right', 'tire_right')
    assert parse_kinematics(no_wheels) is None


def test_missing_laser_returns_none():
    no_laser = MINIMAL_URDF.replace('<child link="base_laser"/>',
                                    '<child link="lidar"/>')
    assert parse_kinematics(no_laser) is None


def test_garbage_returns_none():
    assert parse_kinematics('<robot><unclosed>') is None
    assert parse_kinematics('') is None


def test_malformed_attributes_fall_back_not_crash():
    """可解析但属性畸形（短 xyz / 非数值 radius）必须回退 None 而不是抛异常。"""
    short_xyz = MINIMAL_URDF.replace('xyz="0.15 0.0 0.38"', 'xyz="0.15 0.0"')
    assert parse_kinematics(short_xyz) is None
    bad_radius = MINIMAL_URDF.replace('radius="0.10"', 'radius="abc"')
    assert parse_kinematics(bad_radius) is None
