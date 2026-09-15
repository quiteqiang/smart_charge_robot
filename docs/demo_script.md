# 竞赛演示流程（3–5 分钟）

> 目标观众视角：一台矿卡在仿真园区内自主作业，低电量自动充电后返回任务。
> 全程 RViz2 展示地图/TF/激光/路径/电量；无显示环境用等价命令验证。

## 准备（演示前，不计时）

```bash
cd smart_charge_robot
./scripts/build.sh            # 首次或代码变更后
./scripts/demo.sh             # 无显示服务器；有 X 显示用 ./scripts/demo.sh --rviz
```

等待日志出现 `充电任务状态机就绪 (IDLE)` 与 `Activating bt_navigator`。

## T+0:00 — 启动任务，自主导航

终端 2（`./scripts/dev.sh` 进入同一容器网络，以下 ros2 命令均在其中执行）：

```bash
ros2 service call /mission/start_task std_srvs/srv/Trigger
```

**应看到**：状态机 `IDLE → EXECUTING_TASK`；RViz 中绿色全局路径从 (1,1) 延伸至 work_1 (15,4)，机器人沿路径行驶，红色局部路径滚动刷新；`/battery_state` 电量随行驶缓慢下降。

## T+0:40 — 任务完成 + 动态避障演示

机器人到达 work_1 后自动执行 work_2。趁其行驶中注入障碍物：

```bash
./scripts/spawn_obstacle.sh true     # 或容器内 ros2 topic pub /sim/obstacles ...
```

**应看到**：RViz 中障碍物出现在机器人前方，激光点云打在其上；局部路径绕行（或安全减速停车）。确认后移除：

```bash
./scripts/spawn_obstacle.sh false
```

机器人继续行驶至 work_2，状态回 `IDLE`（全部任务完成）。

## T+1:30 — 低电量自动中断任务

```bash
./scripts/trigger_low_battery.sh 0.20
```

**应看到**（关键高潮，约 2 分钟）：
1. 日志 `低电量 20%，取消当前导航并保存任务` → `LOW_BATTERY → NAVIGATING_TO_DOCK`；
2. 机器人横穿园区驶向充电桩预停靠点 (3.2, 17)；
3. 到达后 `PRE_DOCKING → DOCKING`，机器人以 ≤0.15 m/s 低速对准、缓慢泊入充电桩；
4. 泊靠成功 → 日志 `泊靠成功，请求开始充电` → 电池回报 CHARGING，`CHARGING` 状态，RViz 文本变蓝 “⚡CHARGING”，电量开始回升。

## T+3:30 — 自动恢复任务

```bash
ros2 service call /set_soc smart_charge_msgs/srv/SetSoc "{soc: 0.86}"
```

（自然充电从 25%→85% 约 6 分钟，竞赛演示用服务快进。）

**应看到**：`CHARGING → UNDOCKING`（倒车离桩）→ `RESUMING_TASK` → `EXECUTING_TASK`；机器人自动导航回**之前被暂停的任务航点**并完成，最终 `IDLE`。日志出现 `已恢复被暂停任务`。

## T+4:30 — rosbag 证据回放（可选）

```bash
ros2 bag info bags/demo_run --yaml        # 若启动时 record_bag:=true
ros2 bag play bags/demo_run
```

## 一句话收尾

“低电量检测 → 任务暂停与保存 → 自动导航+精准泊靠 → 充电握手 → 电量恢复 → 离桩 → 任务续做，全链路无人介入；换真车只需替换 4 个硬件接口话题。”
