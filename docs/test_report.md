# smart_charge_robot 测试报告

- 日期：2026-09-16
- 环境：Ubuntu 26.04 宿主机 / Docker `smart_charge_robot:jazzy`（ROS 2 Jazzy desktop + Nav2）
- 硬件：2 vCPU / 1.9 GB RAM（headless，无显示）
- 命令：`./scripts/run_tests.sh`（单容器启动完整系统 + `pytest tests/test_integration.py`，8 场景串行）
- **最终结果：8 passed in 395.46s (0:06:35)** ✅
- 完整日志：`logs/test_run_20260916_052050.log`
- 场景 9 与 Apple Silicon 复验结果见文末「补充验证」一节（本节结果为最初的 8 场景 Linux 宿主机运行）

> 调试过程中共经历 19 轮失败运行，修复了下述真实缺陷后全部通过：
> launch 参数名冲突、rclpy 无 oneshot 定时器、泊靠 yaw 语义定义错误、
> 离桩完成条件符号错误、泊靠末段 bang-bang 转向极限环、任务机泊靠/离桩
> 结果变量名不一致、RPP 参数名误写、symlink-install 下配置需重新构建、
> 低算力启动竞态（就绪重试）等。详见 git 历史与日志。

## 场景明细

### 场景 1：正常导航到作业点 — ✅ PASSED
- 输入：`/mission/goto work_1`
- 预期：`IDLE→EXECUTING_TASK→IDLE`，终点距 work_1 (15,4) ≤0.8 m
- 实际：通过；AMCL 终点位姿距 work_1 0.8 m 内，状态机回 IDLE
- 证据：日志 `[charge_mission] 状态转换: IDLE -> EXECUTING_TASK 执行任务 work_1` → `EXECUTING_TASK -> IDLE 全部任务完成`；bt_navigator `Goal succeeded`

### 场景 2：动态障碍物绕行 — ✅ PASSED
- 输入：机器人驶向 work_2 途中（y≥6），在行进方向前方 3 m、侧向 0.9 m 处注入 0.6 m 方块障碍，40 s 后移除
- 预期：全程与障碍最小距离 >0.5 m（无碰撞）；移除后到达 work_2
- 实际：通过；min_dist > 0.5 m，障碍移除后 `wait_idle_near(work_2)` 满足
- 证据：sim 日志 `动态障碍物加入: Rect(xmin=13.77, ymin=9.06, ...)`；bt_navigator 重规划 BT 周期性 `Passing new path to controller`；最终 `Goal succeeded`

### 场景 3：SOC<25% 切换充电任务 — ✅ PASSED
- 输入：任务执行中 `/set_soc 0.20`
- 预期：取消当前导航并保存任务 → `LOW_BATTERY → NAVIGATING_TO_DOCK`
- 实际：通过
- 证据：`[charge_mission] 低电量 20%，取消当前导航并保存任务 work_1` → `状态转换: EXECUTING_TASK -> LOW_BATTERY` → `LOW_BATTERY -> NAVIGATING_TO_DOCK 规划充电路径`

### 场景 4：进入预停靠区与泊靠区 — ✅ PASSED
- 输入：承接场景 3 自动执行
- 预期：到达 pre_dock (3.2,17) 1 m 内；`PRE_DOCKING → DOCKING`
- 实际：通过
- 证据：`导航 pre_dock [成功] (来源 dock)` → `NAVIGATING_TO_DOCK -> PRE_DOCKING 到达充电桩预停靠点` → `PRE_DOCKING -> DOCKING 第 1 次泊靠尝试`

### 场景 5：泊靠成功后 SOC 上升 — ✅ PASSED
- 输入：进入 CHARGING 后将 charge_rate 调至 0.02/s，观察 15 s
- 预期：`/dock_contact=True`；SOC 上升 >0.15；电池状态 CHARGING
- 实际：通过（SOC 从 ~0.25 升至 ~0.55）
- 证据：`状态转换: DOCKING -> CHARGING 充电桩确认开始充电`；电池节点 `充电请求: False -> True`；`/battery_state` percentage 单调上升

### 场景 6：SOC≥85% 离桩并恢复原任务 — ✅ PASSED
- 输入：`/set_soc 0.86`
- 预期：`UNDOCKING → RESUMING_TASK → EXECUTING_TASK → IDLE`，最终回到 work_1 ≤0.8 m
- 实际：通过
- 证据：`CHARGING -> UNDOCKING 充电完成，离桩` → `DONE: 成功: 离桩完成` → `已恢复被暂停任务: work_1` → `导航 work_1 [成功]` → IDLE 且位姿在 work_1 附近

### 场景 7：桩不可用重试 ≤3 次进入安全错误态 — ✅ PASSED
- 输入：任务执行中屏蔽桩标记（`/sim/dock_visible false`，模拟 AprilTag 感知丢失）+ `/set_soc 0.20`
- 预期：泊靠失败重试 ≤3 次 → `ERROR_WAITING_HUMAN`；`/mission/reset` 复位回 IDLE
- 实际：通过；本场景内 DOCKING 恰好 3 次
- 证据：`[泊靠] FAILED: 失败: 充电桩位姿数据失效` ×3 → `泊靠重试 3 次均失败: 泊靠控制器报告失败` → `DOCKING -> ERROR_WAITING_HUMAN`；`ERROR_WAITING_HUMAN -> IDLE 人工复位`

### 场景 8：rosbag2 录制与回放 — ✅ PASSED
- 输入：`ros2 bag record` 录制 /scan /odom /battery_state /cmd_vel /mission_state 共 12 s
- 预期：`ros2 bag info` 含全部话题且有消息
- 实际：通过；659 条消息（mcap 存储）
- 证据：`bags/test_scenario8/`：`Messages: 659`；`/odom Count: 331`、`/cmd_vel Count: 197`、`/scan Count: 110`、`/battery_state Count: 20`。可用 `ros2 bag play bags/test_scenario8` 回放

### 场景 9：多航点任务队列在低电量中断后完整保留 — ✅ PASSED（独立运行）
- 输入：`/mission/start_task` 载入 [work_1, work_2]；前往 work_1 途中 `/set_soc 0.20`，充满至 0.86 后恢复
- 预期：恢复 work_1 后继续执行 work_2，最终在 work_2 (13,15) ≤0.8 m 处空闲；`EXECUTING_TASK` 进入次数 ≥2
- 实际：独立运行通过（4:44）
- 证据：`UNDOCKING -> RESUMING_TASK 返回被暂停任务 work_1` → `RESUMING_TASK -> EXECUTING_TASK 执行任务 work_2` → `EXECUTING_TASK -> IDLE 全部任务完成`
- **回归有效性验证**：将 `charge_mission_node.py` 回退至修复前（`deb562d~1`），其余环境完全一致，同一场景失败于 `wait_idle_near("work_2")`：
  `AssertionError: 未在 work_2（13.0,15.0）附近空闲，距离 11.15 m`，状态序列为 `RESUMING_TASK -> IDLE 全部任务完成`（work_2 静默丢失）。
  即该用例确实能捕获缺陷，而非无论修复与否都通过的恒真断言。
- 注意：在完整套件顺序执行时本场景失败，原因与本修复无关，见下节。

## 性能与稳定性观察
- 全套 8 场景真实导航时序约 6.5–16 分钟（受 2 vCPU 负载影响波动）。
- 低算力下 Nav2 组件启动存在竞态：`scripts/run_tests.sh` 内置就绪检查（bt_navigator active [3]）与最多 3 次 bringup 重试。
- 内存峰值：bringup + pytest 约 1.2 GB / 1.9 GB，未 OOM。

## 已知限制
- 宿主机无显示：RViz2 未实际渲染（配置已提供）；验证基于话题/日志/TF。
- 泊靠感知为无噪声模拟 AprilTag；未建模相机内参、遮挡、误检。
- 动态障碍物为解析几何模型（矩形/圆柱），非刚体物理；仿真器无自碰撞（贴墙泊靠依赖控制器限速与容差）。
- RPP 的 `use_collision_detection` 会在 footprint 进入障碍膨胀区时冻结机器人（安全停车语义）；狭窄走廊内的极限脱困未做专项测试。
- 单机器人、单层 2D 场景；充电为恒定速率模型；未测试真实 BMS 异常注入（除 SOC 越界）。
- 自动测试为顺序耦合场景（共享同一系统实例），不可乱序/并行执行。

## 补充验证：Apple Silicon（Rosetta amd64 模拟）

- 日期：2026-09-16
- 环境：macOS 26.6.2 / 8 vCPU / 16 GB；Docker Desktop 27.5.1（VM 8 CPU / 8.2 GB）
- 平台：基础镜像 `osrf/ros:jazzy-desktop` 仅发布 amd64，故经 `docker-compose.override.yml` 固定 `platform: linux/amd64`，由 Rosetta 模拟运行（实测 CPU 密集循环约为原生 1.3 倍耗时）
- 镜像构建约 12 分钟；`colcon build` 34 s；bringup 首次即就绪（~2–8 s，未触发 `run_tests.sh` 的重试逻辑）
- 完整套件结果：**9 passed in 949.78s (0:15:49)** ✅（修复 issue #5 之后）
- 修复前同环境结果：8 passed, 1 failed in 1121.22s (0:18:41) —— 场景 9 失败，根因见下
- 日志：`logs/test_run_20260916_165935.log`（9 绿）、`logs/test_run_20260916_160518.log`（修复前）、`logs/s9_withfix.log`、`logs/s9_prefix.log`（均已 gitignore）

场景 9 最初在完整套件中失败，与本分支的队列修复无关，根因有三，均已登记为 issue：

1. **`default_server_timeout: 30`**（`src/smart_charge_navigation/config/nav2_params.yaml:167`）
   —— BT 动作客户端等待服务端确认目标的窗口仅 30 ms，模拟环境下频繁超出，
   出现 `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`；
   因 `max_nav_retries: 2`，两次即进入 `ERROR_WAITING_HUMAN`。见 issue #5。
   **已修复**：该值提升至 1000（本分支）。修复后完整套件 9 绿，且全程日志中
   `acknowledge goal request` 出现 0 次、`连续失败` 0 次。
2. **取消旧目标连带取消新目标** —— 低电量分支在 `cancel_goal_async()` 之后立即下发充电桩目标，
   `navigate_to_pose` 为单目标服务端，取消可能落在新目标上，静默消耗一次导航重试。见 issue #6。
   **未修复**：9 绿那次运行仍出现 2 次 `导航重试 1/2: pre_dock`（场景 3、场景 7 各一次），
   均被重试吸收。修复 #5 后单次重试不再致命，但余量仍然只有一次。
3. **场景 7 遗留状态** —— SOC 停留在 0.20，场景 9 的前置条件窗口被自动触发的充电闭环占用。见 issue #2。
   **未修复**：场景 9 自身的前置处理足以兜住，但该脆弱性仍在。

独立运行（`pytest -k S9`，`default_server_timeout: 1000`，其余一致）：

| 被测节点 | 结果 | 失败点 |
| --- | --- | --- |
| 修复后（`84260b6`） | ✅ 1 passed (4:44) | —— |
| 修复前（`deb562d~1`） | ❌ 1 failed (8:20) | `wait_idle_near("work_2")`，距 work_2 11.15 m |

> 结论：队列修复有效，场景 9 是真实有效的回归用例，完整套件在本环境下 9 绿。
> 仍未处理的 #6、#2 不影响当前结果，但都在消耗容错余量，建议在状态机抽取（#7）时一并解决。
