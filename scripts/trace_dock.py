#!/usr/bin/env python3
"""泊靠调试探针：10 Hz 记录 /dock_relative_pose、/cmd_vel、/docking_status 到 CSV。

用法（容器内）：
  python3 scripts/trace_dock.py 60 | tee /tmp/dock_trace.csv
配合：ros2 service call /mission/reset ... 后重新触发充电流程。
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion


class Trace(Node):
    def __init__(self) -> None:
        super().__init__('dock_trace')
        self.rel = None
        self.cmd = Twist()
        self.status = ''
        self.create_subscription(PoseStamped, '/dock_relative_pose', self._rel, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.create_subscription(String, '/docking_status', self._st, 10)
        self.create_timer(0.1, self._log)

    def _rel(self, m: PoseStamped) -> None:
        q = m.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.rel = (m.pose.position.x, m.pose.position.y, yaw)

    def _cmd(self, m: Twist) -> None:
        self.cmd = m

    def _st(self, m: String) -> None:
        self.status = m.data

    def _log(self) -> None:
        r = self.rel if self.rel else (float('nan'),) * 3
        print(f"{time.monotonic():.3f},{r[0]:.3f},{r[1]:.3f},{r[2]:.3f},"
              f"{self.cmd.linear.x:.3f},{self.cmd.angular.z:.3f},{self.status}",
              flush=True)


def main() -> None:
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = Trace()
    end = time.monotonic() + dur
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
