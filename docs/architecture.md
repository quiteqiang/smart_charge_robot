# smart_charge_robot 架构说明

## 1. 总体决策记录（关键取舍）

| 决策点 | 选择 | 原因 |
|---|---|---|
| 运行环境 | Docker（`osrf/ros:jazzy-desktop` 基镜像，Ubuntu 24.04） | 宿主机为 Ubuntu 26.04，ROS 2 Jazzy 无官方 deb 包 |
| 仿真器 | **自研 minimal simulator**（`mining_truck_sim_node`） | 宿主机仅 1.9 GB RAM / 2 vCPU / 无显示，Gazebo Harmonic + Nav2 + RViz2 无法共存（必然 OOM） |
| 定位 | AMCL + 预构建地图（`slam_toolbox` 仅作可选项保留在镜像中） | MVP 默认已知地图 + 已知起点 |
| 导航 | Nav2（bt_navigator + NavFn 全局规划 + RegulatedPurePursuit 局部控制） | 复用成熟组件，禁止自研规划器 |
| 精准泊靠 | **自研 `dock_controller_node`** | 回退仿真无相机/AprilTag 检测栈，opennav_docking 的检测插件依赖不满足；本节点契约与 Docking Server 对齐，可平滑迁移 |
| 自研节点语言 | Python (rclpy) | 可读性/可测试性优先 |

> ⚠️ 本项目为仿真演示 MVP，不涉及任何真实矿卡、真实充电桩或工程机械安全认证。

## 2. 节点图

```
                         ┌─────────────────────────────┐
                         │      map_server (Nav2)      │  site.pgm 预构建地图
                         └──────────────┬──────────────┘
                                        │ /map
┌──────────────┐   /scan   ┌───────────▼──────────┐   /amcl_pose   ┌──────────────────┐
│ mining_truck │──────────►│  amcl (Nav2)         │───────────────►│ charge_mission   │
│ _sim_node    │           │  map->odom TF        │◄───────────────│ (状态机)          │
│ (替代Gazebo) │           └───┬──────────────┬───┘  /mission/goto │                  │
└─────┬────────┘               │odom->base_link│                  └───┬───────┬──────┘
      │ /odom /imu             ▼              ▼                      │       │
      │                  ┌─────────┐   ┌───────────────┐   action   │       │ /charging_active
      │ /cmd_vel         │base_link│──►│ base_laser    │ NavigateToPose      ▼
      ▼                  │ (URDF)  │   │ imu_link      │──► bt_navigator ◄──┐
┌──────────────┐         └─────────┘   └───────────────┘   controller_server
│  Nav2 栈     │              ▲ robot_state_publisher     planner_server   │
│(bt_navigator │                                              behavior_server
│ controller / │                                                           │
│ planner /    │   泊靠期间接管 /cmd_vel：                                    │
│ behaviors)   │   ┌────────────────┐    /dock_relative_pose               │
└──────▲───────┘   │ dock_controller│◄────────────────── 模拟 AprilTag      │
       │ /cmd_vel  │ _node          │    /dock/start /dock/undock (Trigger)│
       └───────────┤ 对桩低速控制    │    /docking_success /docking_status  │
                   └───────┬────────┘                                      │
                           │ /cmd_vel                                        │
                   ┌───────▼────────┐    /dock_contact    ┌──────────────────┤
                   │ battery_       │◄────────────────────┤ mining_truck_sim │
                   │ simulator_node │──► /battery_state ─►│ (接触判定 0.35m)  │
                   └────────────────┘                     └──────────────────┘
```

## 3. 接口清单

### 话题
| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/scan` | sensor_msgs/LaserScan | sim | AMCL, Nav2 costmaps | 360 束解析射线投射（静态地图+动态障碍） |
| `/odom` | nav_msgs/Odometry | sim | Nav2, 测试 | 差速积分里程计 |
| `/imu` | sensor_msgs/Imu | sim | （预留融合） | |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 / dock_controller | sim | 二者互斥使用（任务机保证时序） |
| `/battery_state` | sensor_msgs/BatteryState | battery | mission, 测试 | percentage + 充放电状态 |
| `/dock_contact` | std_msgs/Bool | sim | battery | 充电枪物理接触（<0.35 m） |
| `/charging_active` | std_msgs/Bool | mission | battery | 任务机“开始充电”握手请求 |
| `/dock_relative_pose` | geometry_msgs/PoseStamped | sim | dock_controller | 桩在 base_link 系下位姿（模拟 AprilTag） |
| `/docking_success` | std_msgs/Bool | dock_controller | mission | TRANSIENT_LOCAL，泊靠结果 |
| `/docking_status` | std_msgs/String | dock_controller | mission/日志 | 泊靠过程文本 |
| `/mission_state` | std_msgs/String | mission | 测试/RViz | 状态机当前状态 |
| `/mission/goto` | std_msgs/String | navigate_to_task/测试 | mission | 单航点任务请求 |
| `/status_text`、`/battery_text_markers` | visualization_msgs/Marker(Array) | mission/battery | RViz | 电量与状态文本 |
| `/visualization_marker_dock` | visualization_msgs/Marker | sim | RViz | 充电桩位置 |
| `/sim/obstacles` | visualization_msgs/Marker | 测试 | sim | 动态障碍物注入（ADD/DELETE） |
| `/sim/dock_visible` | std_msgs/Bool | 测试 | sim | 模拟桩标记感知丢失 |
| `/amcl_pose` | geometry_msgs/PoseStamped | amcl | 测试 | 定位结果 |

### 服务
| 服务 | 类型 | 提供方 | 用途 |
|---|---|---|---|
| `/mission/start_task` | std_srvs/Trigger | mission | 启动任务队列 work_1→work_2 |
| `/mission/reset` | std_srvs/Trigger | mission | ERROR_WAITING_HUMAN 人工复位 |
| `/set_soc` | smart_charge_msgs/SetSoc | battery | 测试注入 SOC |
| `/dock/start`、`/dock/undock` | std_srvs/Trigger | dock_controller | 泊靠/离桩 |

### Action
| Action | 类型 | 客户端→服务端 |
|---|---|---|
| `navigate_to_pose` | nav2_msgs/NavigateToPose | mission → bt_navigator |

### TF 树
```
map ──(amcl)──► odom ──(sim)──► base_link ──(URDF/robot_state_publisher)──► base_laser
                                                        └─► imu_link
```

## 4. 充电任务状态机

```
                 ┌──────────── 低电量 SOC<25%（取消当前导航，保存航点）
                 ▼
IDLE ──start_task──► EXECUTING_TASK ───────► LOW_BATTERY ──► NAVIGATING_TO_DOCK
 ▲  ▲                    ▲   │(导航成功,队列空)                   │ 导航至预停靠点成功
 │  │                    │   ▼                                    ▼
 │  │                    │  IDLE(全部任务完成)                 PRE_DOCKING ──► DOCKING
 │  │                    │                                       ▲            │ /dock/start
 │  │                    └────────── RESUMING_TASK ◄── UNDOCKING ◄┘            │ 成功+握手
 │  │                                              ▲   ▲(SOC≥85%)    ┌───────▼──────┐
 │  │                                              │   └── CHARGING ◄─┤ 充电桩确认    │
 │  │             导航失败重试耗尽 / 泊靠重试耗尽 / 定位丢失 / 电量异常   └──────────────┘
 │  │                    │                                                   │ 泊靠失败
 │  └────────────────────┴──────────────► ERROR_WAITING_HUMAN ◄──重试<3次─────┘
 │                                           │ /mission/reset
 └───────────────────────────────────────────┘
```

安全约束：
- 泊靠失败退回预停靠点重试，**最多 3 次**（`max_docking_retries`），耗尽进入 `ERROR_WAITING_HUMAN`，不会无限循环；
- 导航失败重试 `max_nav_retries=2` 次；
- 定位看门狗：`map->base_link` TF 丢失超过 `tf_loss_timeout_s=5s` 进入错误态；
- 电量数据异常（NaN/越界）立即进入错误态；
- 错误态仅允许人工 `/mission/reset` 复位。

## 5. 精准泊靠控制序列（dock_controller）

```
/dock/start ──► ALIGN（原地旋转对准桩方向，|Δyaw|<0.15）
            ──► APPROACH（比例控制 v=k·d，线速上限 0.15 m/s；横向偏差>0.4rad 先转向）
            ──► FINAL（d<0.35 m 进入末段，线速上限 0.05 m/s，容差 x/y 0.04 m、yaw 0.06 rad）
            ──► 成功：/docking_success=True
安全：前向 ±60° 激光 <0.18 m 急停失败；位姿数据失效 >1 s 失败；总超时 45 s。
离桩：/dock/undock 直线倒车 0.8 m（线速 0.12 m/s，超时 30 s）。
```

## 6. 关键参数

| 参数（文件） | 默认值 | 含义 |
|---|---|---|
| `low_soc_threshold` | 0.25 | 低电量阈值（mission + battery 同步配置） |
| `resume_soc_threshold` | 0.85 | 恢复任务阈值 |
| `charge_rate` | 0.01 /s | 充电速率（测试可调大加速） |
| `discharge_rate` / `idle_discharge_rate` | 0.002 / 0.0001 /s | 行驶/待机放电速率 |
| `max_docking_retries` | 3 | 泊靠最大重试 |
| `dock_success_timeout_s` | 60 s | 等待泊靠结果总超时 |
| `charger_connect_distance`（sim） | 0.35 m | 充电枪接触判定距离 |
| Nav2 速度 | `desired_linear_vel 0.45`，robot_radius 0.32 | 低速安全演示 |
| `set_initial_pose`（amcl） | (1.0, 1.0, 0.0) | 与仿真起点一致 |

## 7. 替换为真实硬件的接口边界

| 仿真组件 | 真实替换方式 |
|---|---|
| `mining_truck_sim_node` | 真实底盘驱动订阅 `/cmd_vel` 并发布 `/odom`、`/imu`；激光雷达驱动发布 `/scan` |
| `/dock_relative_pose` | AprilTag/二维码检测节点发布同一话题（frame=base_link） |
| `/dock_contact` | 充电桩/BMS 的物理连接信号 |
| `battery_simulator_node` | 真实 BMS 驱动发布 `/battery_state`；`/set_soc` 服务仅测试用 |
| sim 动态障碍物/AprilTag 可见性 | 不需要（测试专用） |

Nav2、AMCL、mission 状态机、dock_controller 全部无需改动。
