#!/usr/bin/env python3
"""任务导航 CLI：把机器人导航至指定航点（经 mission 状态机，尊重低电量打断）。

用法：
  ros2 run smart_charge_mission navigate_to_task work_1
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    if len(sys.argv) < 2:
        print('用法: navigate_to_task <waypoint_name>')
        sys.exit(1)
    name = sys.argv[1]

    rclpy.init()
    node = Node('navigate_to_task_cli')
    pub = node.create_publisher(String, '/mission/goto', 10)
    msg = String()
    msg.data = name
    # 等待 mission 节点订阅建立
    for _ in range(20):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f'已请求导航至航点: {name}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
