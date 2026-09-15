"""smart_charge_robot 端到端集成测试（场景 1-8）。

前提：完整系统已通过 full_demo.launch.py 启动（scripts/run_tests.sh 负责）。
测试以 rclpy 客户端身份驱动真实系统：发布任务、注入 SOC、注入/移除动态
障碍物、屏蔽充电桩标记，并断言状态机转换、位姿、电量与 rosbag 产物。

注意：场景按顺序共享同一系统实例，顺序不可重排。
"""
from __future__ import annotations

import math
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from smart_charge_msgs.srv import SetSoc
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

# 航点真值（与 smart_charge_navigation/config/waypoints.yaml 一致）
WAYPOINTS = {
    "work_1": (15.0, 4.0),
    "work_2": (13.0, 15.0),
    "pre_dock": (3.2, 17.0),
    "dock": (1.2, 17.0),
}
STATES = [
    "IDLE", "EXECUTING_TASK", "LOW_BATTERY", "NAVIGATING_TO_DOCK",
    "PRE_DOCKING", "DOCKING", "CHARGING", "UNDOCKING",
    "RESUMING_TASK", "ERROR_WAITING_HUMAN",
]


class SystemProbe(Node):
    """订阅关键话题、提供等待原语。"""

    def __init__(self) -> None:
        super().__init__("system_probe")
        self.state = "IDLE"
        self.state_history: list[tuple[float, str]] = []
        self.pose = (0.0, 0.0)          # /amcl_pose (map)
        self.odom_pose = (0.0, 0.0)
        self.soc = 1.0
        self.battery_status = 0
        self.cmd_lin_x = 0.0
        self.dock_contact = False

        self.create_subscription(String, "/mission_state", self._on_state, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.create_subscription(Bool, "/dock_contact", self._on_contact, 10)

        self.goto_pub = self.create_publisher(String, "/mission/goto", 10)
        self.obstacle_pub = self.create_publisher(Marker, "/sim/obstacles", 10)
        self.dock_vis_pub = self.create_publisher(Bool, "/sim/dock_visible", 10)

        self.start_cli = self.create_client(Trigger, "/mission/start_task")
        self.reset_cli = self.create_client(Trigger, "/mission/reset")
        self.set_soc_cli = self.create_client(SetSoc, "/set_soc")

    # ---- 回调 ----
    def _on_state(self, msg: String) -> None:
        if msg.data != self.state:
            self.state = msg.data
            self.state_history.append((time.monotonic(), msg.data))
            print(f"[probe] state -> {msg.data}")

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.cmd_lin_x = msg.twist.twist.linear.x

    def _on_battery(self, msg: BatteryState) -> None:
        self.soc = msg.percentage
        self.battery_status = msg.power_supply_status

    def _on_contact(self, msg: Bool) -> None:
        self.dock_contact = msg.data

    # ---- 原语 ----
    def wait_state(self, target: str, timeout: float) -> None:
        """等待目标状态出现（含已进入过的瞬态：基于历史而非当前值轮询）。"""
        self.wait_state_in([target], timeout)

    def wait_state_in(self, targets: list[str], timeout: float) -> str:
        seen = set(s for _, s in self.state_history)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for s in targets:
                if s in seen:
                    return s
            time.sleep(0.2)
            seen = set(s for _, s in self.state_history)
        raise AssertionError(f"等待状态集合 {targets} 超时（历史 {sorted(seen)}）")

    def wait_idle_near(self, wp: str, tol: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        tx, ty = WAYPOINTS[wp]
        while time.monotonic() < deadline:
            if self.state == "IDLE":
                d = math.hypot(self.pose[0] - tx, self.pose[1] - ty)
                if d <= tol:
                    return
            time.sleep(0.5)
        d = math.hypot(self.pose[0] - tx, self.pose[1] - ty)
        raise AssertionError(f"未在 {wp}（{tx},{ty}）附近空闲，距离 {d:.2f} m")

    def _call(self, cli, request, timeout: float = 15.0):
        """异步调用 + 轮询：executor 由后台 spin 线程驱动，
        不可与 rclpy.spin_until_future_complete 混用（会抛 Executor is already spinning）。"""
        assert cli.wait_for_service(timeout_sec=10.0)
        future = cli.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        assert future.done(), "服务调用超时"
        return future.result()

    def call_trigger(self, name: str) -> None:
        cli = {"start": self.start_cli, "reset": self.reset_cli}[name]
        res = self._call(cli, Trigger.Request())
        assert res.success, f"{name} 调用失败: {res.message}"

    def set_soc(self, soc: float) -> None:
        req = SetSoc.Request()
        req.soc = soc
        res = self._call(self.set_soc_cli, req)
        assert res.success, f"set_soc 失败: {res.message}"

    def goto(self, wp: str) -> None:
        for _ in range(5):
            self.goto_pub.publish(String(data=wp))
            time.sleep(0.2)

    def set_obstacle(self, x: float, y: float, size: float, present: bool) -> None:
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "dyn"
        m.id = 1
        m.type = Marker.CUBE
        m.action = Marker.ADD if present else Marker.DELETE
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = size
        m.scale.z = 0.8
        for _ in range(5):
            self.obstacle_pub.publish(m)
            time.sleep(0.2)

    def set_dock_visible(self, visible: bool) -> None:
        for _ in range(5):
            self.dock_vis_pub.publish(Bool(data=visible))
            time.sleep(0.2)

    def count_state_entries(self, target: str, since: float = 0.0) -> int:
        return sum(1 for t, s in self.state_history if s == target and t >= since)

    def dist_to(self, x: float, y: float) -> float:
        return math.hypot(self.pose[0] - x, self.pose[1] - y)


# ---------------------------------------------------------------- 全局夹具
probe: SystemProbe | None = None
_spin_thread: threading.Thread | None = None


def _spin() -> None:
    while rclpy.ok():
        rclpy.spin_once(probe, timeout_sec=0.1)


@pytest.fixture(scope="module", autouse=True)
def system_probe():
    global probe, _spin_thread
    rclpy.init()
    probe = SystemProbe()
    _spin_thread = threading.Thread(target=_spin, daemon=True)
    _spin_thread.start()
    # 等待 mission 订阅 /mission/goto（DDS 发现可能需要数秒，过早发布会丢消息）
    deadline = time.monotonic() + 20.0
    while probe.goto_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.5)
    assert probe.goto_pub.get_subscription_count() > 0, "mission 未订阅 /mission/goto"
    yield probe
    probe.destroy_node()
    rclpy.shutdown()


def set_battery_param(name: str, value: float) -> None:
    subprocess.run(
        ["ros2", "param", "set", "/battery_simulator", name, str(value)],
        check=True, capture_output=True, timeout=30)


# ---------------------------------------------------------------- 场景
class TestS1_NormalNavigation:
    def test_goto_work_1(self):
        """场景1：正常导航到作业点 work_1。"""
        probe.goto("work_1")
        probe.wait_state("EXECUTING_TASK", 30)
        probe.wait_idle_near("work_1", tol=0.8, timeout=180)


class TestS2_ObstacleAvoidance:
    def test_dynamic_obstacle_on_route(self):
        """场景2：行进前方侧向注入障碍物 -> 重规划绕行（无碰撞），移除后到达。"""
        probe.goto("work_2")
        probe.wait_state("EXECUTING_TASK", 30)

        # 等机器人向北推进（离开 work_1 区域）
        deadline = time.monotonic() + 120
        while probe.pose[1] < 6.0 and time.monotonic() < deadline:
            time.sleep(0.5)
        assert probe.pose[1] >= 6.0, f"机器人未向北推进（当前 y={probe.pose[1]:.2f}）"

        # 在行进方向前方 3 m、侧向偏移 0.9 m 处投放 0.6 m 方块：
        # 挡住原路径的一半走廊，机器人必须重规划绕行（RPP 碰撞检测不会冻结）。
        tx, ty = WAYPOINTS["work_2"]
        px, py = probe.pose
        d = math.hypot(tx - px, ty - py)
        ux, uy = (tx - px) / d, (ty - py) / d
        OX, OY = px + ux * 3.0 - uy * 0.9, py + uy * 3.0 + ux * 0.9
        probe.set_obstacle(OX, OY, 0.6, present=True)
        time.sleep(1.0)

        min_dist = float("inf")
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            min_dist = min(min_dist, probe.dist_to(OX, OY))
            time.sleep(0.3)
        probe.set_obstacle(OX, OY, 0.6, present=False)

        # 无碰撞：最小距离 > 机器人半径(0.32) + 障碍物半宽(0.3)
        assert min_dist > 0.5, f"与障碍物最小距离 {min_dist:.2f} m，疑似碰撞"
        probe.wait_idle_near("work_2", tol=0.8, timeout=300)


class TestS3_LowBatteryInterrupt:
    def test_low_soc_triggers_charge_mission(self):
        """场景3：SOC<25% 时暂停任务并转充电任务。"""
        probe.goto("work_1")
        probe.wait_state("EXECUTING_TASK", 30)
        # 确认机器人在移动（任务执行中）
        time.sleep(3.0)
        # 放慢充电速率：避免任务机在测试到达 CHARGING 前自动充到 0.85 离桩
        set_battery_param("charge_rate", 0.002)
        probe.set_soc(0.20)
        probe.wait_state("LOW_BATTERY", 30)
        probe.wait_state_in(["NAVIGATING_TO_DOCK"], 60)


class TestS4_PreDockAndDocking:
    def test_reach_pre_dock_and_dock(self):
        """场景4：进入预停靠区，随后执行精准泊靠。"""
        probe.wait_state("PRE_DOCKING", 420)
        assert probe.dist_to(*WAYPOINTS["pre_dock"]) < 1.0, \
            f"未到达预停靠区，距离 {probe.dist_to(*WAYPOINTS['pre_dock']):.2f} m"
        probe.wait_state("DOCKING", 150)


class TestS5_ChargingRaisesSoc:
    def test_charging_raises_soc(self):
        """场景5：泊靠成功后 SOC 上升（加速充电速率以缩短测试）。"""
        probe.wait_state("CHARGING", 200)
        # dock_contact 可能因 DDS 发现延迟晚到：轮询等待
        deadline = time.monotonic() + 15
        while not probe.dock_contact and time.monotonic() < deadline:
            time.sleep(0.3)
        assert probe.dock_contact, "已进入充电态但 dock_contact 为 False"
        # 0.02/s × 15s ≈ +0.30：足以验证上升，又不会越过 0.85 自动离桩
        set_battery_param("charge_rate", 0.02)
        soc0 = probe.soc
        time.sleep(15.0)
        assert probe.soc > soc0 + 0.15, f"SOC 未明显上升: {soc0:.2f} -> {probe.soc:.2f}"
        set_battery_param("charge_rate", 0.002)
        assert probe.battery_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING


class TestS6_ResumeTask:
    def test_full_charge_resumes_saved_task(self):
        """场景6：SOC>=85% 离桩并恢复被暂停的任务航点。"""
        probe.set_soc(0.86)
        set_battery_param("charge_rate", 0.01)   # 复位默认充电速率
        probe.wait_state_in(["UNDOCKING"], 30)
        probe.wait_state_in(["RESUMING_TASK"], 60)
        probe.wait_state_in(["EXECUTING_TASK"], 120)
        probe.wait_idle_near("work_1", tol=0.8, timeout=240)


class TestS7_DockFailureRetries:
    def test_dock_unavailable_enters_error(self):
        """场景7：充电桩不可见（感知丢失）-> 泊靠重试至多 3 次 -> 安全错误态。"""
        t0 = time.monotonic()
        probe.goto("work_2")
        probe.wait_state("EXECUTING_TASK", 30)
        time.sleep(2.0)
        probe.set_dock_visible(False)
        time.sleep(1.0)
        try:
            probe.set_soc(0.20)
            probe.wait_state("ERROR_WAITING_HUMAN", 400)
            n = probe.count_state_entries("DOCKING", since=t0)
            assert 1 <= n <= 3, f"泊靠尝试次数 {n} 超出 [1,3]"
        finally:
            probe.set_dock_visible(True)
        # 人工复位后应回 IDLE
        probe.call_trigger("reset")
        probe.wait_state("IDLE", 30)


class TestS8_RosbagRecord:
    def test_rosbag_record_and_info(self):
        """场景8：rosbag2 录制关键话题并可查询。"""
        out_dir = "bags/test_scenario8"
        topics = ["/scan", "/odom", "/battery_state", "/cmd_vel", "/mission_state"]
        rec = subprocess.Popen(
            ["ros2", "bag", "record", "-o", out_dir] + topics,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(12.0)
        finally:
            rec.terminate()
            rec.wait(timeout=30)
        info = subprocess.run(
            ["ros2", "bag", "info", out_dir],
            check=True, capture_output=True, text=True, timeout=60)
        # Jazzy ros2 bag info 默认输出人类可读表格（Topic: /x | Count: n）
        for t in topics:
            assert f"Topic: {t} " in info.stdout or f"Topic: {t}|" in info.stdout or f"Topic: {t}\n" in info.stdout, \
                f"bag 中缺少话题 {t}"
        assert "Messages:" in info.stdout
