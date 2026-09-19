# smart_charge_robot 改进方向分析

> 分析日期：2026-09-16 · 基于 `develop` 分支代码逐文件深读
> （mission 状态机 / battery 仿真 / docking 控制器 / sim 节点 / launch / nav2 参数 / 集成测试）
> 进度更新（2026-09-19）：P0 #1、#2 已修复并过 `/code-review medium`
> （分支 `fix/p0-dock-pose-loss-and-scan-perf`，review 发现 1 项 Low 已修）；
> P1 #3、#4 已修复（review 发现 2 项 Medium + 2 项 Low 已全部修复，状态机单测 38 例全绿）；
> P1 #5（CI）、#6（电池模型）已修复（review 发现 1 项 Medium + 1 项 Low 已修，
> 单测累计 56 例全绿）

## 总体评价

项目当前形态是一个**完成度很高的 MVP**：闭环功能（导航 → 低电量暂停 → 泊靠 → 充电 → 恢复任务）已跑通，有架构文档、8 场景集成测试、Docker 一键复现，代码注释质量高于一般 ROS 2 教学项目。状态机的防护意识（结果与状态不匹配则忽略、TF 看门狗、指令超时、结构化日志）值得肯定。

以下改进按 **优先级 × 主题** 组织。P0 = 正确性/安全隐患，P1 = 可维护性/可扩展性，P2 = 锦上添花。

---

## P0 — 正确性与安全

### 1. 泊靠控制器对 `dock_rel` 的解包未判空（`dock_controller_node.py:155`）

> ✅ **已修复**（2026-09-19，commit `a6aacbe`）：控制器不再因单帧 age 超阈值瞬时判失败，
> 改为连续丢失容忍——新增 `pose_lost_tolerance_s`（默认 0.5s，按 control_hz 折算周期数），
> 短暂丢失期间刹车等待数据恢复，累计超限才 `_finish(False)`；计数在收到新数据及
> dock/undock 启动时重置。同时 sim 端 `/dock_relative_pose` 由 5 Hz 提到 10 Hz，
> 对 1.0s 的 `pose_timeout_s` 留出更大抖动裕度。场景 7（感知丢失→重试 3 次→错误态）语义不变，
> 仅失败判定从"瞬时 1s"变为"约 1.5s 累计"。

```python
x, y, dyaw = self.dock_rel      # 第 74 行只声明了类型 Optional
```

前面第 151 行确实检查了 `self.dock_rel is None`，但检查在 `if` 里、`return` 之后解包——逻辑上没问题；真正的隐患是 `pose_timeout_s=1.0` 而 sim 端 `/dock_relative_pose` 只发 5 Hz：正常时 age ≤ 0.2s 没问题，但**仿真负载高时一帧延迟 + jitter 就可能触达 1s 超时**。超时时直接判失败而不是"短暂丢失后重试"，会把瞬时的感知抖动放大成整个泊靠失败（虽然 mission 层有 3 次重试兜底）。

**改进**：
- 位姿丢失容忍改为"连续 N 个控制周期丢失才失败"（如 20Hz × 0.5s），而不是瞬时 age 比较；
- 或者 sim 端把 dock pose 提到 10-20 Hz（计算量极小），同时控制器加死区。

### 2. sim 节点激光计算的 O(n·m) 开销（`mining_truck_sim_node.py:299-305`）

> ✅ **已修复**（2026-09-19，commit `e618edd` + review 修复 `789528f`）：静态世界加载时转为
> (R,4)/(P,3) numpy 数组，每帧光束方向由缓存角度数组旋转变换得到（消除逐 beam 三角函数），
> slab/圆求交整块的 (B,R)/(B,P) 数组运算完成（`raycast_static_np`）。实测当前地图
> （7 rect + 1 pillar）**3.8× 提速**（0.75ms→0.20ms/帧），且随世界增大差距线性拉大
> （标量路径随物体数线性增长，向量化路径基本平坦）。已向量化/标量两路做对拍验证
> （3000 随机世界 × 180 光束 + 矩形内/柱内/贴墙边界扫描，0 不一致）。
> `msg.ranges` 初始化同步改为 `inf`（顺手修掉 #13 表中对应行）。
> numpy 为**可选依赖**（Dockerfile 与 package.xml 已加 `python3-numpy`）：缺 numpy 的瘦主机
> 自动回退原纯 Python 路径。动态障碍物（通常为空/单个 rect）仍逐条标量求交，不付广播开销。
> review 发现的"缓存参数 ros2 param set 不生效"问题已通过 `on_set_parameters` 回调修复。

每帧 180 beam，每条 beam 对 static_world 的每条 rect + pillar 做解析求交。**每帧 ~数百次求交，纯 Python**。目前 8 Hz 在小世界能撑住，但：world 变大、beam 变密、或跑在 1.9GB 瘦主机上 CPU 受限时，scan 周期会被拉长 → AMCL 更新延迟 → 定位抖动 → 连锁影响导航。且 `_publish_scan` 里 `msg.ranges = [0.0] * beams` 后又逐元素填充，`r < r_min` 时填 `inf`——注意 **0.0 是非法值但有些消费者当 0 处理**，初始 0.0 若未被覆盖（不会，循环全覆盖）倒是没问题，但语义上应初始化为 `inf` 更稳。

**改进**：
- beam 求交加 **空间网格索引 / numpy 向量化**（把 rect/pillar 转成 numpy 数组一次性 slab 求交），预计 10-50× 提速；
- 或用 `array('f')` / 预分配复用消息；
- 中期可切 Gazebo（README 说内存不允许）或更轻的 physics（如 Box2D 风格的 pure-python 库）——非必须。

---

## P1 — 可维护性 / 可扩展性

### 3. 阈值参数三处重复定义

`low_soc_threshold` / `resume_soc_threshold` 同时出现在：
- `charge_mission_node.py`（declare_parameter）
- `battery_simulator_node.py`（declare_parameter，用于 LOW! 提示）
- `battery_params.yaml` / `mission_params.yaml`

**风险**：改一个不改另两个，会出现"电池节点认为 25% 是低电量但状态机 20% 才动作"的不一致。电池节点其实并不需要这两个阈值（提示文本可用 mission 状态替代，或直接订阅 `/mission_state`）。

**改进**：阈值只由 mission 节点持有并发布（参数事件 `/parameter_events` 或 `/charge_policy` 话题），电池节点被动显示。

> ✅ **已修复**（2026-09-19，commit `ec27ef5` + review 修复 `91260fb`）：电池节点删除
> `low_soc_threshold` / `resume_soc_threshold` 副本（含 `battery_params.yaml`），
> `[LOW!]` 提示改由订阅 latched `/mission_state` 推导（充电循环状态 = LOW_BATTERY /
> NAVIGATING_TO_DOCK / PRE_DOCKING / DOCKING）。mission 节点成为唯一决策持有者。
> review 发现的降级场景（mission 离线/人工错误态时不显示低电量警告）用独立的
> display-only 参数 `low_soc_display_threshold`（默认 0.10，刻意低于决策阈值，
> 不参与决策）兜底。docs/architecture.md 参数表已同步。

### 4. 缺少单元测试，只有端到端集成测试

`tests/test_integration.py` 是 8 场景串行集成测试（12-18 分钟，README 自述顺序不可重排）。这带来两个结构性问题：

- **反馈环太长**：改一行状态机代码要等 15 分钟才知道有没有破坏场景 6；
- **场景耦合**：`顺序不可重排` = 共享隐藏状态，加一个场景可能破坏旧场景，且无法并行跑。

**改进**：
- 把状态机核心抽成 **不依赖 rclpy 的纯 Python 类**（输入：电池事件/导航结果/服务调用；输出：状态+动作），用 pytest 单测覆盖全部转换边（含错误路径、重试耗尽、reset）。rclpy 薄壳只做 IO。这是本项目**性价比最高的重构**——状态机逻辑 ~300 行，纯化后可测性质变；
- 泊靠控制器的几何/控制律（`normalize_angle`、曲率修正、各阶段转换）同理可抽纯函数单测；
- 集成测试保留 2-3 个冒烟场景即可，其余下放到单测。

> ✅ **状态机纯化已完成**（2026-09-19，commit `fc147cb` + review 修复 `91260fb`）：
> `mission_logic.MissionStateMachine`（纯 Python，不依赖 rclpy）持有全部状态/决策，
> 副作用以 Effect 列表交回 `charge_mission_node.py` 薄壳执行；ROS 对外行为与原实现
> 逐回调核对一致。`src/smart_charge_mission/test/test_mission_logic.py` **38 个 pytest
> 用例**覆盖全部转换边（正常充电循环、低电量暂停/恢复、resume_none、导航/泊靠重试
> 耗尽、服务拒绝、等待超时、latched 旧结果拒绝、控制器重启世代递增、孤儿 oneshot
> 防护、goto 语义、非法 SOC、TF 丢失、reset、迟到服务响应防护），<0.1s 跑完。
> review 发现并修复：参数运行时调参失效（`on_set_parameters` → `update_config`）、
> `dock/undock` 迟到失败响应可把机器拖出错误态（加状态守卫）。
> **仍开放**：泊靠控制器控制律单测、集成测试精简为冒烟集、#5 CI（build+单测）。

### 5. CI 缺失

> ✅ **已修复**（2026-09-19，commit `e3b0803` + review 修复 `e7b633f`）：新增
> `.github/workflows/ci.yml`。PR/每次推送：dev 镜像构建（gha 层缓存）→
> `pytest src`（纯 Python 单测，秒级，无需 ROS）→ `colcon build`；每晚 cron +
> 手动触发跑完整集成测试（`scripts/run_tests.sh`，9 场景）。容器内以 root 运行
> 避开 GH runner uid(1001) 与 compose 用户 (1000) 的挂载属主冲突；nightly 任务
> 先 `chown -R 1000:1000` 让 ros2bag 能写回挂载目录。review 发现的"fork PR 的
> cache-to 无写权限会挂构建"已用条件表达式修复（仅同仓库 PR/推送导出缓存）。
> lint（ruff/mypy）暂未纳入——存量代码未跑过 lint，先保证 CI 常绿，留作后续。

有 Docker（`docker-compose.yml` + `scripts/build.sh`）却没有 `.github/workflows/`。集成测试贵（15 分钟），但 **build + 单测（若做了 #4）+ lint** 应该每次 PR 跑。

**改进**：
- GitHub Action：container build → `colcon build` → `colcon test`（或 pytest 单测）→ 每晚跑一次完整集成测试（可配 `schedule` 触发）；
- 加 `ruff`/`flake8` + `mypy --strict`（代码已大量用类型注解，收紧成本很低）。

### 6. 电池模型过于理想化，限制了仿真价值

> ✅ **已修复**（2026-09-19，commit `c3c8f9a` + review 修复 `e7b633f`）：电池模型
> 抽为纯 Python 的 `BatteryModel`（`battery_model.py`，不依赖 rclpy，16 例单测），
> 节点退化为 ROS IO 薄壳，与 #4 状态机同一模式。已实现：① CC/CV 两段充电
> （`cc_cv_threshold` 默认 0.8，以下恒流满速，以上线性降速到 0——resume 阈值
> 0.85 落在 CV 段，"充到 85% 要多久"现在更真实）；② 端电压 = OCV(soc) − |I|·R，
> 一阶滞后逼近（`voltage_tau_s`），`/battery_state` 新增 `current` 字段；③ 内部
> Ah 记账，SOC = charge/capacity。顺手修复原实现缺陷：满电挂桩时旧代码掉入放电
> 分支造成"满电振荡"。review 发现的 `capacity_ah` 无校验已补 ValueError。
> ROS 接口/发布频率不变，`ros2 param set charge_rate` 实时调参路径保持可用。

`battery_simulator_node.py` 的模型：SOC 线性充放电，电压 = 22 + 4·SOC（注释自述"示例"）。没有：
- 内阻/压降（大电流行驶时电压下垂）；
- 温度影响；
- 容量衰减；
- 充电曲线的 CC/CV 两段（恒流快充→恒压涓流）。

**改进**（按性价比排序）：
1. 充电改为 CC/CV 两段：`SOC < 0.8` 时 `charge_rate` 满速，之后线性降速到 0——只用 3 行就能让"充到 85% 需要多久"的仿真更真实；
2. 电压加一阶滞后 + 负载项：`v = OCV(soc) - I·R`，让 `/battery_state.voltage` 不再和 SOC 完全线性相关（更接近真实 BMS 输出，也为将来接真实 BMS 做接口对齐）；
3. `capacity=100.0` 硬编码且 `percentage` 与 `capacity` 无耦合，建议 `percentage = charge / capacity` 的真实 bookkeeping。

### 7. `/mission/goto` 抢占语义与文档/服务的不一致

`_on_goto` 允许在 EXECUTING_TASK 中直接抢占当前导航目标，但 `_on_start_task` 只在 IDLE 接受。结果是：**外部系统可以通过 topic 抢占，却不能通过更正式的服务接口抢占**——权限倒挂。另外抢占时 `saved_task` 不更新，低电量中断后恢复到的是**被抢占前的旧目标**还是新目标，取决于时序，存在歧义。

**改进**：明确抢占语义（"goto = 追加到队列头部"还是"替换当前任务"），并保证 `saved_task` 在任何抢占路径下都指向用户最后意图的航点。

### 8. URDF 与 sim 参数无单一事实源

`mining_truck.urdf`（47 行）与 `sim_params.yaml` 中的 `wheel_radius: 0.10`、`wheel_separation: 0.42` 是两份手工同步的数值；`laser_x: 0.15` 只存在于 sim 参数，URDF 里 base_laser 位置要单独核对。改一个忘改另一个，robot_state_publisher 发布的 TF 就和仿真真值漂移。

**改进**：让 sim 节点从 URDF 解析 wheel separation / laser pose（已有 `robot_state_publisher` 在跑，用 `tf2` 查 `base_link → base_laser` 即可拿到 laser_x），只保留 URDF 一份。

---

## P2 — 锦上添花

### 9. rosbag 录制写死相对路径

`full_demo.launch.py` 里 `-o bags/demo_run` 是相对路径——依赖 launch 时的工作目录。在 systemd/robot 上启动时可能写到意想不到的地方，且同名覆盖无提示。

**改进**：用 `LaunchConfiguration` + 默认值指向 `$HOME/.ros/bags/` 或带时间戳的路径；或用 `rosbag2` 的 `--max-bag-size` 做滚动。

### 10. AMCL 初始位姿写死、无重定位服务

`nav2_params.yaml` 里 `set_initial_pose: true, initial_pose: (1.0, 1.0, 0.0)`——与 `waypoints.yaml` 的 `start` 一致，但仍是两处手工同步。且 sim 支持 `/sim/reset_pose`，AMCL 却不知道（TF 看门狗会发现定位丢失然后进错误态——这其实是**正确行为**，但没有"重新初始化定位"的恢复路径，复位后 AMCL 粒子仍可能收敛回旧位姿）。

**改进**：launch 里用 `waypoints.yaml` 的 start 生成 AMCL 初始位姿（或反过来）；提供 `/relocalize` 服务：调 AMCL `reinitialize_global_localization` + 用 sim 真值重置（demo/调试体验提升明显）。

### 11. 日志中文化

状态机日志 `"状态转换: ..."`、`"低电量..."` 等是中文。对国内团队没问题，但 README 是英文为主、GitHub 公开——英文日志（或双语 key）对国际化和日志聚类工具（ELK 等按 token 分词）更友好。**优先级低**，看项目定位。

### 12. 对接真实硬件的路径未产品化

代码里已埋好接口注释（"真实硬件替换方式：停用本节点，将真实 BMS 驱动发布到相同话题即可"），但没有：
- 真实 BMS/AprilTag 驱动的参考实现或 adapter 包；
- `dock_relative_pose` 的真实感知节点（目前只有 sim 的"完美感知"——无噪声、无丢帧模型，只有 `/sim/dock_visible` 开关）。

**改进**：给 `/dock_relative_pose` 加高斯噪声 + 丢帧模拟（sim 内 5 行），让泊靠控制在"脏数据"下也被测试覆盖——这是上真实硬件前最便宜的置信度来源。

### 13. 小项清单

| 位置 | 问题 | 改动 |
|---|---|---|
| `charge_mission_node.py` `_on_battery` | NaN/越界 SOC 直接 `_enter_error`，单帧毛刺就杀掉整个任务 | 连续 N 帧（如 5 帧 @2Hz）异常才进错误态 |
| `charge_mission_node.py` 模块级 | `import yaml` 无异常处理，waypoints 文件不存在时裸 traceback | try/except + 清晰错误信息 + 非零退出 |
| `battery_simulator_node.py` 函数内 import | 函数内 `from geometry_msgs.msg import ...`，风格不一致 | 移到模块顶部 |
| `mining_truck_sim_node.py` `_publish_scan` | ~~`msg.ranges` 初始 `0.0` 应为 `inf`~~ ✅ 已修（`e618edd`，随 #2 一并完成） | 一行 |
| `dock_controller_node.py` | `_poll_dock_result` 递归 oneshot 创建/销毁定时器链，线程安全依赖 rclpy 单线程 executor 假设 | 文档化该假设，或改为单一定时器 + 状态判断 |
| 各节点 | `get_parameter(...).value` 在 20Hz 控制循环内反复调用（dock_controller 每个周期读 10+ 次参数） | `__init__` 缓存；确实需要运行时调的（charge_rate 有注释说明）才保留动态读 |

---

## 建议实施顺序

```
第 1 步（已完成）：任务队列保留（#7）、结果通道竞态修复（DockResult 世代校验）、
                  LOW_BATTERY 状态语义、reset 上下文清理
第 2 步（已完成）：#4 状态机纯 Python 化 ✅、#5 CI ✅（2026-09-19，build+单测
                  分钟级反馈，集成测试每晚跑）——改动反馈从 15 分钟降到秒级
第 3 步（按需）：  #1 ✅、#3 ✅、#6 ✅（均 2026-09-19 完成）、#8 TF 单源化
第 4 步（上硬件前）：#12 感知噪声模拟 + 真实 BMS/AprilTag adapter
```

## 一个总体观察

这个项目的**主要技术债不在任何单个 bug，而在于"所有验证都靠在完整系统上跑 15 分钟集成测试"**。状态机、泊靠控制律、电池模型、sim 几何——四块逻辑全部埋在 rclpy 回调里，耦合了 ROS 通信层。把核心业务逻辑抽成纯 Python（不 import rclpy），是当前投入产出比最高的一步：它同时解决测试反馈慢（#4）、CI 难建（#5）、重构不敢做（连锁放大所有其他项）三个问题。
