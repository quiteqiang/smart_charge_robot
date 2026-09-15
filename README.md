# smart_charge_robot

无硬件、可仿真演示的工程机械（矿卡）自主导航与自动充电 MVP。
ROS 2 Jazzy + Nav2 + AMCL + 自研 minimal simulator，Docker 一键复现。

> 本项目仅用于仿真演示，不涉及真实矿卡、真实充电桩或任何工程机械安全认证。

## 功能闭环

起点 → 自主导航至作业航点（局部避障）→ SOC < 25% 自动暂停任务 → 导航至充电桩预停靠点 → 低速精准泊靠 → 充电握手 → SOC ≥ 85% 离桩 → **返回并恢复被暂停的任务**。
RViz2 展示地图/TF/激光/全局与局部路径/电量/充电状态/充电桩；支持 rosbag2 录制与自动测试。

## 环境要求

- Docker ≥ 24 与 Docker Compose（**必须**；宿主机不需要预装 ROS/Gazebo）
- 推荐 ≥ 2 vCPU / 2 GB RAM / 5 GB 磁盘（1.9 GB 内存的瘦主机已验证可运行，不开 RViz）
- 宿主机可以是任意 Linux；开发基准为 ROS 2 Jazzy @ Ubuntu 24.04 容器

### 为什么不原生安装 / 为什么不用 Gazebo

- 宿主机若不是 Ubuntu 24.04（如 Ubuntu 26.04），ROS 2 Jazzy 无官方 deb 包 → Docker 为唯一可行方案。
- Gazebo Harmonic + Nav2 + RViz2 需 4 GB+ 内存，1.9 GB 瘦主机必然 OOM → 采用
  **Nav2 兼容的最小仿真器**（差速运动学 + 解析射线激光 + 模拟 AprilTag），
  Nav2/AMCL/任务栈 100% 为真实组件。详见 `docs/architecture.md` 决策记录。

## 快速开始（Docker，唯一需要的路径）

```bash
git clone <repo> smart_charge_robot && cd smart_charge_robot
cp .env.example .env                     # 可选

# 构建镜像（首次；基于 osrf/ros:jazzy-desktop + Nav2 等）
docker compose build                     # 或使用会话中已构建的 smart_charge_robot:jazzy

./scripts/build.sh                       # colcon build（容器内）
./scripts/demo.sh                        # 一键完整演示（headless）
./scripts/demo.sh --rviz                 # 有 X 显示时开启 RViz2
```

启动后另开终端 `./scripts/dev.sh` 进入同一容器网络执行 ros2 命令。

### 演示操作（详见 docs/demo_script.md）

```bash
ros2 service call /mission/start_task std_srvs/srv/Trigger   # 开始任务队列
./scripts/trigger_low_battery.sh 0.20                        # 注入低电量 -> 自动充电闭环
ros2 service call /set_soc smart_charge_msgs/srv/SetSoc "{soc: 0.86}"   # 快进充满 -> 恢复任务
ros2 topic echo /battery_state                               # 观察电量
```

## RViz2 观察

`./scripts/demo.sh --rviz`（需 X 显示，如 `ssh -X`）。预设 `rviz/smart_charge.rviz` 包含：
Map、TF、RobotModel、LaserScan、全局/局部 Costmap、全局路径 `/plan`、局部路径 `/local_plan`、
充电桩标记 `/visualization_marker_dock`、电量与状态文本 `/status_text` + `/battery_text_markers`。
Headless 机器可用 `ros2 topic echo /status_text`（Marker 文本）与 `/mission_state` 等价观察。

## 测试

```bash
./scripts/run_tests.sh      # 启动完整系统并运行 tests/test_integration.py（8 个场景，约 12-18 分钟）
```

场景：①正常导航 ②动态障碍无碰撞通过 ③低电量切换充电任务 ④预停靠+泊靠
⑤充电 SOC 上升 ⑥充满恢复任务 ⑦桩不可用重试≤3次进错误态 ⑧rosbag2 录制回放。
结果记录在 `docs/test_report.md`。

## rosbag2

```bash
./scripts/record_bag.sh                          # 录制关键话题到 bags/run_<时间戳>/
ros2 launch smart_charge_bringup full_demo.launch.py record_bag:=true
ros2 bag info bags/run_xxxx --yaml && ros2 bag play bags/run_xxxx
```

## 目录结构

```
smart_charge_robot/
├── docker/Dockerfile, ../docker-compose.yml   # 复现环境
├── scripts/                                   # build/demo/test/record/触发脚本
├── src/                                       # 7 个包 + msgs
│   ├── smart_charge_base/                     # URDF 矿车模型 + 静态 TF
│   ├── smart_charge_simulation/               # minimal simulator（Gazebo 替代）
│   ├── smart_charge_battery/                  # 电池 SOC 仿真
│   ├── smart_charge_mission/                  # 充电任务状态机 + navigate_to_task
│   ├── smart_charge_docking/                  # 精准泊靠控制器（Docking Server 替代）
│   ├── smart_charge_navigation/               # Nav2/AMCL 配置、地图、航点
│   ├── smart_charge_bringup/                  # 一键 launch + RViz 预设
│   └── smart_charge_msgs/                     # SetSoc / AddObstacle 服务
├── maps/                                      # site.pgm/yaml、world.yaml（脚本生成）
├── launch/ config/                            # 顶层便捷入口（指向包内文件）
├── tests/test_integration.py                  # 端到端集成测试
├── bags/ docs/                                # 录制输出 / 架构、演示、测试报告
```

## 替换为真实硬件

替换 4 个接口即可，Nav2/AMCL/状态机/泊靠控制器均无需改动（详见 `docs/architecture.md` §7）：
`/cmd_vel`+`/odom`+`/imu`+`/scan`（底盘与雷达驱动）、`/dock_relative_pose`（AprilTag 检测）、
`/dock_contact`（充电桩连接信号）、`/battery_state`（BMS）。

## 已知限制

- Gazebo/RViz 在 ≤2 GB 内存主机上不可用（RViz 配置已提供，仅缺显示资源）；
- 泊靠感知为无噪声模拟 AprilTag，未做相机内参/遮挡建模；
- 单机器人、单层 2D 场景；充电功率为恒定速率模型；
- 动态障碍物为矩形/圆柱解析模型，非刚体物理；
- 自动测试全链路约 12-18 分钟（真实导航时序，未做加速回放）。
# smart_charge_robot
