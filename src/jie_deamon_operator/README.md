# deamon / jie_deamon — M20 Pro 跟随与远程控制

云深处 **山猫 M20 Pro** 四足机器人的二维雷达跟随 + 手机/Android 远程控制网关。

本仓库在 `bernardleex526/deamon2`（D1 跟随算法 + M20 `basic_server` 速度桥）之上，
增加了统一的操作员网关（网页/Android 独占控制权），并**清除 2026-09-08 实机排查中发现的
全部硬阻塞**：使用模式冲突、步态最小速度与限幅矛盾、横向速度算法缺陷、自体点导致无法
arm、root/组播 relay 运维依赖。

> **状态（2026-09-09）**：代码层阻塞已全部清除；算法与门控已在本机 WSL（Ubuntu 22.04 +
> ROS 2 Humble）用「可移动模拟人」12 个场景验证通过（12/12），跟随 smoke 与操作员网关
> smoke 均通过。
> **但实机跟随仍未验收**：从未向 AOS 发送过一条速度指令，`commissioned:=true` 从未执行。
> 任何 live 测试前必须先完成 [最小验收 checklist](docs/M20_ACCEPTANCE_CHECKLIST.md)。

---

## 目录

- [1. 它做什么 / 不做什么](#1-它做什么--不做什么)
- [2. 系统架构与数据链路](#2-系统架构与数据链路)
- [3. 运行环境与前置条件](#3-运行环境与前置条件)
- [4. 快速开始（GOS 上）](#4-快速开始gos-上)
- [5. 安装与编译](#5-安装与编译)
- [6. 配置参考](#6-配置参考)
- [7. 控制协议与门控逻辑](#7-控制协议与门控逻辑)
- [8. 操作流程 SOP](#8-操作流程-sop)
- [9. 手机 / Android 操作入口](#9-手机--android-操作入口)
- [10. 诊断与排障](#10-诊断与排障)
- [11. 测试与验证](#11-测试与验证)
- [12. 雷达选型与坐标系](#12-雷达选型与坐标系)
- [13. 安全边界与未完成项](#13-安全边界与未完成项)
- [14. 文档索引](#14-文档索引)
- [15. 来源与许可证](#15-来源与许可证)

---

## 1. 它做什么 / 不做什么

**做**：

- 订阅 GOS 上厂商融合点云 `/LIDAR/POINTS`，投影为 2D 扫描 `/m20/scan`（前向 ±100° 扇区）。
- 用 D1 跟随算法（卡尔曼 + 走廊 + 势场避障）计算跟随速度 `/m20/cmd_vel_raw`。
- 通过 fail-closed 门控输出 `/m20/cmd_vel_guarded`，可选经 AOS `basic_server` UDP 下发速度。
- 提供**唯一控制权**的操作员网关：网页/Android 认证 + 序号防重放 + 心跳租约，断连即停。
- 提供只读预检、传感器探针、状态探针、合成人形仿真等工具。

**不做**（明确边界）：

- 不是 SLAM / 定位 / 路径规划 / 地图 / 充电程序 —— 这些仍由 NOS 的原厂 `drmap` 负责。
- 不是四足运控 —— AOS 的 `rl_deploy` 负责步态与关节控制，本仓库只通过公开接口发速度。
- 不做姿态/步态动作（站立、趴下、切步态、D1 动作）—— 一律拒绝，继续用原厂控制器。
- 不替代安全机制 —— 软件零速是「减速停车请求」，**不是**软急停（软急停=关节断电）。

---

## 2. 系统架构与数据链路

### 2.1 主机职责

| 主机 | 文档默认地址 | 职责 | 本仓库的处置 |
| --- | --- | --- | --- |
| AOS | `10.21.31.103` | 运控 `rl_deploy`、`basic_server` | 只通过公开接口通信，不安装、不修改 |
| NOS | `10.21.31.106` | 雷达驱动、SLAM、规划、充电、PTP Master | 不安装；建图仍用原厂 `drmap` |
| GOS | `10.21.31.104` | 用户二次开发主机 | **安装本仓库** |

三台主机均为 RK3588（8 核、16 GB RAM）。官方建议二次开发程序绑大核：`taskset -c 4-7`。

### 2.2 数据链路

```text
前后雷达原始组播（33 网段）
  -> NOS multicast-relay（仅转发原始 UDP）
  -> GOS 厂商解码与融合
  -> /LIDAR/POINTS        PointCloud2，机身坐标（frame_id=lidar_link）
  -> pointcloud_to_laserscan -> /m20/scan   前向 ±100° 2D 扫描
  -> robot_nexus（D1 跟随算法）
  -> /m20/cmd_vel_raw    算法意图（无 operator 时）
     └─ 或 /robot_nexus/operator_velocity（带 session 标记，operator follow 模式）
  -> m20_bridge 门控（状态/扫描/指令新鲜度 + 步态速度区间 + 停止区）
  -> /m20/cmd_vel_guarded  预览（dry-run）或实际下发（live）
  -> AOS basic_server UDP 30000：Type=2 Cmd=25（导航模式，m/s）
                              或 Type=2 Cmd=21（常规模式，归一化）
  <- Type=1002 Cmd=6 BasicStatus（2 Hz 主动推送，需先发心跳 Type=100 Cmd=100）

手机 HTTP 8080 / Android UDP 8889
  -> 认证 + 序号防重放 -> 独占操作员租约（OperatorControl）-> 同一个门控
```

**关键点**：`/LIDAR/POINTS` 由 GOS 厂商驱动解码并**已在驱动层转换到 `base_link`**，
但 `dds_frame_id` 标签仍是 `lidar_link`。因此 `pointcloud_to_laserscan` 的
`target_frame` 必须为空字符串，**不要**再做一次 TF 变换，也**不要**改
`send_separately`（改成独立模式会导致原厂导航/定位不可用、自主充电部分受限）。

---

## 3. 运行环境与前置条件

| 项目 | 要求 |
| --- | --- |
| 机器人固件 | **≥ V1.1.7**（GOS 点云输出、`multicast-relay` 支持）；建议 **V1.1.8**（修复「开机后没有点云」「外部跨版本 ROS2 监听导致内部 ROS2 崩溃」） |
| GOS 系统 | Ubuntu 20.04 + ROS 2 **Foxy** + DrDDS（Fast DDS 封装） |
| RMW | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| Domain | `ROS_DOMAIN_ID=0`，`ROS_LOCALHOST_ONLY=0` |
| 依赖包 | `pointcloud_to_laserscan`、`libopencv-dev`、`ament_cmake_python`、`ament_cmake_gtest`、`std_srvs` |
| 权限 | **root**：官方明确 `/LIDAR/POINTS` 必须 `su` 才能取到数据 |
| 服务 | `multicast-relay.service` 必须 **active + enabled**（出厂默认关闭） |
| 网络 | AOS `10.21.31.103:30000`（UDP）/ `:30001`（TCP）可达 |

环境准备（每次验收都从干净 root 环境开始）：

```bash
sudo -i
export ROS_DISTRO=foxy
source /opt/robot/scripts/setup_ros2.sh
source /home/user/m20_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

> 注意：厂商 `setup_ros2.sh` 可能改写 `/opt/robot/fastdds.xml`。只做只读检查时，
> 可以改用 `source /opt/ros/foxy/setup.bash` + 手动导出上面的变量。

---

## 4. 快速开始（GOS 上）

```bash
# 1) 一次性：放到工作区并编译
cd ~/m20_ws/src
git clone https://github.com/bernardleex526/deamon.git jie_deamon
cd ~/m20_ws
source /opt/robot/scripts/setup_ros2.sh
colcon build --packages-select jie_deamon
source install/setup.bash

# 2) 只读预检（12 项检查，写 /tmp/m20_preflight.json）
ros2 run jie_deamon m20_preflight.py --seconds 12 --require-scan
echo "exit=$?"        # 必须是 0

# 3) 演练（不打开 AOS 控制 socket，不 arm）
cd ~/m20_ws/src/jie_deamon
bash scripts/m20ctl preview

# 4) 另一个终端取手机口令
bash scripts/m20ctl token
#    手机浏览器打开 http://<GOS 可达 IP>:8080 ，粘贴口令，选点后按 Start
```

实机入口（**仅在完成验收与原厂控制权移交之后**）：

```bash
bash scripts/m20ctl live --commissioned
```

`m20ctl` 会自动：切到 root → 校验 NOS relay 服务 → 跑点云探针 → 生成/校验口令文件 →
以 `flock` 防止重复启动 → 启动 launch。

---

## 5. 安装与编译

### 5.1 依赖

```bash
source /opt/ros/foxy/setup.bash
sudo apt-get update
sudo apt-get install -y libopencv-dev python3-colcon-common-extensions \
  ros-${ROS_DISTRO}-ament-cmake-python ros-${ROS_DISTRO}-ament-cmake-gtest \
  ros-${ROS_DISTRO}-ament-lint-auto ros-${ROS_DISTRO}-ament-lint-common \
  ros-${ROS_DISTRO}-pointcloud-to-laserscan
```

### 5.2 编译与测试

```bash
cd ~/m20_ws
colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select jie_deamon --ctest-args -R test_m20_tracker --output-on-failure
colcon test-result --verbose
cd src/jie_deamon && python3 -m unittest discover -s test -v
```

> 全量 `ament` lint 在既有代码上并非全绿（上游遗留版权/风格问题与 vendored
> `httplib.h` 工具语言报错）。本仓库只保证**行为测试**与聚焦的 C++ 测试通过。

### 5.3 无机器人本地演练（WSL / 开发机）

```bash
source /opt/ros/humble/setup.bash      # 或 foxy
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 合成点云 + 状态
ros2 launch jie_deamon m20.launch.py cloud_topic:=/m20/mock_points enable_web:=false
ros2 run jie_deamon m20_mock_inputs

# 或：可移动模拟人 + 12 个断言场景（推荐）
ros2 run jie_deamon m20_follow_sim --report /tmp/m20_follow_sim.json
```

`m20_follow_sim` 启动自己的 launch，全程 dry-run，不接触 AOS。它强制隔离：
未设置 ROS 环境时自动使用 `ROS_DOMAIN_ID=83` + `ROS_LOCALHOST_ONLY=1`；
显式指定 `ROS_DOMAIN_ID=0`（机器人域）或 `ROS_LOCALHOST_ONLY=0` 会被拒绝。

---

## 6. 配置参考

### 6.1 `config/m20.yaml`

**`robot_nexus`（跟随算法）**

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `active` | `true` | 启动跟随计算 |
| `enable_web` / `enable_android` / `enable_actions` | `false` | 上游 D1 的 Web/Android/动作服务器，M20 全部关闭 |
| `enable_opencv` | `false` | OpenCV 可视化窗口（GOS 无显示器，保持关闭） |
| `scan_yaw` | `0.0` | 扫描旋转角。D1 用 π，M20 用 0（点云已在机身坐标） |
| `follow_distance` | `1.2` | 跟随距离（m） |
| `corridor_width` | `0.8` | 走廊宽度（m），用于目标筛选与横向居中 |
| `frame_front/back/left/right` | `0.41/0.41/0.253/0.253` | 机身排除框（M20 机身 0.82 × 0.506 m） |
| `direct_timeout` | `0.3` | 直接控制指令有效期（s） |
| `enable_lateral` | **`false`** | 横向走廊居中。**默认关闭**：D1 原式在单侧墙时输出约 ±0.7 m/s |
| `lateral_gain` / `lateral_max` / `lateral_deadband` / `lateral_sign` | `0.5 / 0.3 / 0.05 / 1.0` | 仅在 `enable_lateral=true` 时生效 |
| `min_forward_speed` | `0.0` | 前后最小速度注入；`0` 表示交给门控按步态下限处理 |

**`m20_cloud_to_scan`（点云转扫描）**

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `target_frame` | `''` | **必须为空**：融合点云已是机身坐标，再做 TF 会二次变换 |
| `min_height` / `max_height` | `-0.1 / 0.5` | 2D 切片高度带（相对机身原点） |
| `angle_min` / `angle_max` | `±1.7453293`（±100°） | 前向扇区，几何上等价「只用前雷达的前向区域」 |
| `angle_increment` | `0.0087266`（0.5°） | 约 401 bins |
| `range_min` / `range_max` | `0.1 / 12.0` | 量程 |
| `use_inf` / `concurrency_level` | `true / 1` | 单线程，避免与厂商进程抢核 |

**`m20_bridge`（门控与下发）**

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `dry_run` | `true` | `true` 绝不打开 AOS 控制 socket |
| `commissioned` | `false` | live 启动必须显式置 `true`（操作者声明） |
| `aos_host` / `aos_port` | `10.21.31.103 / 30000` | AOS `basic_server` |
| `control_profile` | `auto` | `auto` \| `si`（仅 `Cmd=25`）\| `normalized`（仅 `Cmd=21`） |
| `allow_auxiliary_mode` | `false` | 是否允许辅助模式（=2）下发 |
| `max_vx/max_vy/max_wz` | `0.3 / 0.3 / 0.6` | 安全限幅（SI 单位） |
| `axis_enable_x/y/yaw` | `true/true/true` | 逐轴开关；`axis_enable_y:=false` 可彻底禁用横移 |
| `allow_reverse` | `false` | **算法**输出是否允许后退（±100° 扇区看不到后方） |
| `allow_reverse_operator` | `true` | **操作员**遥控是否允许后退（手机后退键） |
| `gait_min_policy` | `zero` | `zero`：低于步态下限置零（fail-closed）；`snap`：抬到下限 |
| `min_request` | `0.02` | 噪声门限，小于此值的请求视为 0 |
| `axis_max_x/y/yaw` | `2.0 / 1.0 / 2.0` | `Cmd=21` 归一化满量程，**需实机标定** |
| `stop_front/back/half_width` | `0.7 / 0.7 / 0.4` | 停止区（m） |
| `stop_hits_required` | `1` | 连续多少帧命中才判障碍（2~3 可抑制单帧噪声） |
| `self_front/back/left/right` | `0.41/0.41/0.253/0.253` | 自体遮罩机身框 |
| `self_margin_front/back/left/right` | `0.05/0.14/0.07/0.07` | 逐侧余量（后/侧覆盖腿部扫掠） |
| `scan_min_range` | `0.05` | 小于此距离的回波忽略 |
| `guard_sector_min/max` | `±1.7453293` | 门控只看前向 ±100° |
| `min_sector_bins` | `20` | 扇区有效回波不足即判无效（**前雷达失效检测**） |
| `require_preflight` | `true` | live 启动要求新鲜、全通过的预检报告 |
| `preflight_path` | `/tmp/m20_preflight.json` | 预检报告路径 |
| `preflight_max_age` | `600` | 报告有效期（s） |
| `control_hz` | `20.0` | 下发频率（官方建议 ≥20 Hz） |

### 6.2 `launch/m20.launch.py` 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `cloud_topic` | `/LIDAR/POINTS` | 输入点云（仿真时用 `/m20/mock_points`） |
| `dry_run` | `true` | 不打开控制 socket |
| `commissioned` | `false` | live 必须显式 `true` |
| `enable_web` | `true` | 启用操作员网关（8080/8889）；`false` 走纯 ROS 路径 |
| `operator_token_file` | `$M20_OPERATOR_TOKEN_FILE` | 口令文件路径 |
| `operator_host/port` | `0.0.0.0 / 8080` | 网页监听 |
| `android_port` | `8889` | Android UDP 端口（`0` 关闭） |
| `control_profile` | `auto` | 见上 |
| `axis_enable_y` / `axis_enable_yaw` | `true / true` | 逐轴开关 |
| `allow_reverse` | `false` | 算法后退 |
| `gait_min_policy` | `zero` | 步态下限策略 |
| `require_preflight` | `true` | live 预检闸门 |
| `use_sim_time` | `false` | 回放 bag 时用 `true`（live 拒绝仿真时间） |

---

## 7. 控制协议与门控逻辑

### 7.1 为什么有两种速度指令

官方文档明确：

- `Type=2 Cmd=25`（SI：m/s、rad/s）**仅在导航模式下可用**。
- `Type=2 Cmd=21`（归一化 [-1,1]，相对该轴最大速度）**仅在常规/辅助模式下可用**。

出厂默认使用模式是**常规模式（`ControlUsageMode=0`）**，所以旧实现硬编码要求
`ControlUsageMode==1` 会导致门控永远不放行。现在按实时模式自动选择：

| `control_profile` | `ControlUsageMode` | 下发命令 | 单位 |
| --- | --- | --- | --- |
| `auto`（默认） | 1 导航 | `Cmd=25` | m/s、rad/s |
| `auto`（默认） | 0 常规 | `Cmd=21` | 归一化 |
| `auto`（默认） | 2 辅助 | 拒绝（需 `allow_auxiliary_mode:=true`） | — |
| `si` | 仅 1 | `Cmd=25` | m/s |
| `normalized` | 仅 0 | `Cmd=21` | 归一化 |

**结论：不需要为了用本仓库而把手柄/机器人切到导航模式。** 常规模式下用 `Cmd=21`，
与手柄共用同一轴指令通道 —— 因此跟随期间操作员必须松手不要碰摇杆。
切到导航模式是可选项，代价是**步态被重置为 0x1001**，且导航模式会启用原厂导航栈
（必须先停 `planner.service`）。

### 7.2 步态有效速度区间（官方 V1.1.7+）

速度区间是**分段**的，零点附近不可用：

| 步态 | 十六进制 | X (m/s) | Y (m/s) | Yaw (rad/s) |
| --- | --- | --- | --- | --- |
| 标准-基础 | `0x1001`/4097 | [0.20, 2.0] | [0.35, 1.0] | [0.50, 2.0] |
| 标准-楼梯 | `0x1003`/4099 | [0.15, 2.0] | [0.30, 1.0] | [0.40, 2.0] |
| 敏捷-平地 | `0x3002`/12290 | [0.15, 2.0] | [0.25, 1.0] | [0.35, 1.5] |
| 敏捷-楼梯 | `0x3003`/12291 | [0.15, 2.0] | [0.30, 1.0] | [0.40, 2.0] |

门控按**实时步态**处理每个轴：

| 条件 | `zero`（默认） | `snap`（可选） |
| --- | --- | --- |
| `\|v\| < min_request` | 0 | 0 |
| `min_request ≤ \|v\| < 步态下限` | 0 | 抬到步态下限 |
| 步态下限 ≤ `\|v\|` ≤ `min(限幅, 步态上限)` | 原值 | 原值 |
| 步态下限 > `min(限幅, 步态上限)` | 0，并写 `axis_notes` | 同左 |

> 副作用：`zero` 策略下，误差小于「步态下限 ÷ 比例系数」时机器人**完全不动**，
> 一超过就以步态下限起步。要平滑起步用 `snap`，但必须先在实机接受最小速度响应。

### 7.3 门控放行条件（全部满足才输出非零）

1. 已 `arm`（操作员显式使能，障碍/超时后**不会自动恢复**）。
2. `BasicStatus` 新鲜（< 1.5 s，适配 2 Hz 上报）且完整。
3. `MotionState=17`（RL 控制）、`Direction=0`、`HES=0`、`Charge=0`、`Sleep=0`、步态已知。
4. 使用模式与 `control_profile` 兼容（见 7.1）。
5. 扫描新鲜（< 0.5 s）、frame 正确、扇区有效回波 ≥ `min_sector_bins`、停止区无回波。
6. 速度指令新鲜（< 0.3 s）；纯 ROS 路径下 `arm()` 还要求速度流**已经存在**。
7. 每个轴通过步态速度区间量化与限幅。

任一条件失效 → 立即解除使能并输出零速，`/m20/bridge_status` 记录原因。

### 7.4 通信细节

- 帧头 `EB 91 EB 90` + uint16 小端 ASDU 长度 + 报文 ID + JSON 标志 1 + 7 字节保留 = 16 字节，
  后接 UTF-8 JSON `PatrolDevice`；ID 从 0 递增到 65535 回绕。
- 心跳 `Type=100 Cmd=100` 每秒一次，之后 AOS 才推送 BasicStatus（2 Hz）。
- 同一个已连接 socket 用于发送与接收，保持源端口不变。
- 零错误 ACK 只代表接收，不代表动作完成；本仓库依赖完整 BasicStatus 而不是 ACK 放行。
- 机器人侧超时保护：**500 ms** 未收到新速度指令会自动减速进入安全状态（文档约定，需实测）。

---

## 8. 操作流程 SOP

> 完整可执行清单见 [docs/M20_ACCEPTANCE_CHECKLIST.md](docs/M20_ACCEPTANCE_CHECKLIST.md)。

1. **只读检查**：确认固件版本、`multicast-relay.service` 状态、点云频率与 frame。
2. **预检**：`ros2 run jie_deamon m20_preflight.py --seconds 12 --require-scan`，
   required 检查必须全绿（root、relay、planner 空闲、无 `/NAV_CMD` 发布者、点云/扫描新鲜、
   AOS 心跳与 BasicStatus、步态可用）。
3. **dry-run**：`bash scripts/m20ctl preview`，观察 `/m20/scan_report`、`/m20/bridge_status`，
   确认清空场地时 `stop_hits=0`、`sector_bins` 正常。
4. **单轴最小速度验证**：先 `max_vx:=0.1 max_wz:=0.2`，逐轴、逐符号验证单位与方向；
   标定 `axis_max_*`。
5. **首次 live（不 arm）**：`dry_run:=false commissioned:=true`，确认只有心跳与零速，
   机器人静止 30 s。
6. **低速度跟随**：选定目标 → `set_moving` / 手机 Start → 逐项验证前进、保持、左右转、
   目标丢失、障碍解除。
7. **故障演练**：停算法、停桥、杀进程、断点云、断网、硬/软急停、切手动、充电态、
   遮挡单雷达 —— 每项记录**实测停车距离**。
8. **恢复**：disarm → 确认静止 → 退出 launch → 按记录恢复原厂 planner/模式/步态。

---

## 9. 手机 / Android 操作入口

```bash
cd /home/user/m20_ws/src/jie_deamon
bash scripts/m20ctl preview          # 演练（不打开控制 socket）
bash scripts/m20ctl token            # 取口令
bash scripts/m20ctl status           # 只读状态
bash scripts/m20ctl stop             # 停止租约
bash scripts/m20ctl live --commissioned   # 实机（验收后）
```

- 手机浏览器打开 `http://<GOS 可达 IP>:8080`，粘贴口令；支持雷达选点、按住方向控制、
  开始跟随、停止，以及状态/拦截原因显示。
- **唯一控制权**：网页与 Android 共用同一个 `OperatorControl` 租约，只有一名操作员可驱动；
  其他客户端可以 STOP，但不能接管。
- 认证与防重放：`Bearer` 口令 + 每客户端单调递增序号；序号回退或重复即拒绝。
- 心跳租约：默认 0.6 s 无输入即超时停机；**断连后不自动恢复**，必须重新按 Start。
- Android 手机可直接用网页；原生 App 需实现认证/序号/心跳协议，旧 D1 App 不兼容。

---

## 10. 诊断与排障

### 10.1 关键话题

| 话题 | 内容 |
| --- | --- |
| `/m20/bridge_status` | JSON：`armed`、`reason`、`command_kind`、`usage_mode`、`gait`、`axis_notes`、`velocity`、`counters`、`since_command/scan/status`、`scan` 摘要 |
| `/m20/scan_report` | JSON：`valid`、`clear`、`valid_bins`、`sector_bins`、`stop_hits`、`self_hits`、`age`、`closest_stop`、`closest_self`、`reason` |
| `/m20/cmd_vel_raw` | 算法意图（未经门控） |
| `/m20/cmd_vel_guarded` | 门控输出（dry-run 仅预览） |

### 10.2 常见 `reason` 与处置

| `reason` | 含义 | 处置 |
| --- | --- | --- |
| `not armed` | 未使能 | 手机 Start 或 `ros2 service call /m20/arm` |
| `no fresh command stream` | 纯 ROS 路径下 arm 时速度流未就绪 | 确认 `robot_nexus` 在发布 `/m20/cmd_vel_raw`，重试 arm |
| `command stale` | 0.3 s 无新速度指令 | 算法/网关停止输出；检查进程与租约 |
| `scan stale` / `scan unavailable` | 扫描超时或无效 | 查点云、relay、`min_sector_bins` |
| `invalid scan or obstacle` | 停止区命中或扫描无效（**latch**） | 清空障碍后**手动重新 arm** |
| `usage mode N (…) incompatible with control profile auto` | 模式与档位冲突 | 用 `control_profile:=auto` 或切模式 |
| `not RL control` / `estop, charging or sleeping` | 状态不允许 | 原厂控制端起立/解除急停/等待充电空闲 |
| `unsupported gait` | 步态不在表内 | 记录并更新 `GAIT_TABLE` |
| `BasicStatus stale` | 状态上报中断 | 检查 AOS 心跳与网络 |
| `UDP failure: …` | 网络错误 | 查路由与 AOS 服务 |
| `basic_server rejected command: {…}` | AOS 返回 `ErrorCode != 0` | 读错误码（官方错误码文档） |

### 10.3 传感器排障顺序

```bash
systemctl status multicast-relay.service --no-pager
systemctl status rsdriver.service --no-pager
journalctl -u rsdriver.service -n 40 --no-pager      # 看 ptp=0x1、前后雷达 status
ros2 run jie_deamon m20_cloud_probe.py --seconds 12 --cloud-only
ros2 run jie_deamon m20_status_probe.py --seconds 6  # 只发心跳，不发速度
ros2 topic echo --once /m20/scan_report
```

---

## 11. 测试与验证

### 11.1 分层测试

| 层 | 命令 | 覆盖 |
| --- | --- | --- |
| 单元（无 ROS） | `python3 -m unittest discover -s test -v` | 协议编解码、门控、模式矩阵、步态区间、自车遮罩、扇区、预检闸门、操作员租约、HTTP 服务器 |
| C++ | `colcon test --ctest-args -R test_m20_tracker` | 跟踪器行为、横向居中双侧约束、直接控制超时 |
| ROS smoke | `python3 test/ros_m20_smoke.py` | 默认零速、前进跟随、目标丢失、障碍锁定、输入中断 |
| 操作员 smoke | `python3 test/ros_operator_smoke.py` | HTTP 选点 → 带标记跟随预览 → 断连停机 |
| 建图观察 smoke | `python3 test/ros_m20_mapping_smoke.py` | 被动观察器不产生速度发布者 |
| **仿真** | `ros2 run jie_deamon m20_follow_sim` | 可移动模拟人 12 个场景，断言 raw/guarded |

### 11.2 本机（WSL Ubuntu 22.04 + ROS 2 Humble）实测结果

```text
colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=ON   # exit 0
python3 -m unittest discover -s test                                        # Ran 71 tests ... OK
colcon test --ctest-args -R test_m20_tracker                                # 5 tests, 0 failures
python3 test/ros_m20_smoke.py                                               # PASS
python3 test/ros_operator_smoke.py                                          # PASS
python3 -m m20_adapter.follow_sim                                           # 12/12 scenarios passed
```

仿真场景（`docs/evidence/m20_follow_sim_report.json`）：

| # | 场景 | raw | guarded | 结论 |
| --- | --- | --- | --- | --- |
| 01 | 人在正前 2.0 m | (0.4, 0, 0) | (0.3, 0, 0) | 前进 |
| 02 | 人在跟随距离 1.2 m | (0, 0, 0) | (0, 0, 0) | 保持 |
| 03 | 人退到 0.9 m | (−0.12, 0, 0) | (0, 0, 0) | 算法后退被门控禁止 |
| 04/05 | 人移到左/右前 | wz ±0.78 | wz ±0.6 | 转向符号正确、限幅生效 |
| 06 | 小偏角 | (0.10, 0, 0.24) | (0, 0, 0) | 低于步态下限 → fail-closed |
| 07 | 目标消失 | 0 | 0 | 不追残留目标 |
| 08 | 加入真实自车回波 | (0.4, 0, 0) | (0.3, 0, 0) | 不再误停，可 arm |
| 09 | 正前 0.5 m 障碍 | (0.4, 0, 0) | (0, 0, 0) | 解除使能 |
| 10 | 后方 −0.6 m 障碍 | (0.4, 0, 0) | (0.3, 0, 0) | 扇区外不误触发 |
| 11 | 常规模式（`ControlUsageMode=0`） | (0.4, 0, 0) | (0.3, 0, 0) | `Cmd=21` 路径可用 |
| 12 | 导航模式（`=1`） | (0.4, 0, 0) | (0.3, 0, 0) | `Cmd=25` 路径可用 |

> Humble 只验证「算法 + 门控 + 网关逻辑」，**不能替代 Foxy/ARM64 与实机验收**。

---

## 12. 雷达选型与坐标系

完整分析见 [docs/LIDAR_CHOICE_ANALYSIS.md](docs/LIDAR_CHOICE_ANALYSIS.md)。

**结论：保留厂商融合点云，但把有效视场收窄到前向 ±100°**，不修改 `send_separately`。

- 扇区内 `pointcloud_to_laserscan` 每个 bin 只保留最近点，而前雷达对同一物体距离恒小于
  后雷达，因此扇区内胜出者必为前雷达 —— 几何上等价「只用前雷达的前向区域」。
- 修改 `send_separately: true` 会让官方导航/定位不可用、自主充电部分受限，故不做。
- 前雷达失效检测：`sector_bins < min_sector_bins` 即 fail-closed；进一步可用
  `Type=1002 Cmd=5` 的 `DevEnable.Lidar.Front != 1` 主动 disarm（待现场接入）。
- 坐标系：点云已在 `base_link`，但 `frame_id` 标签是 `lidar_link`；
  `target_frame: ''` 保留物理坐标，**不要**额外平移 ±0.32028 m。

---

## 13. 安全边界与未完成项

### 13.1 安全边界（不可妥协）

- 绝不与 `drmap mapping`、原厂导航、充电或其它速度控制器同时运行。
- `dry_run:=true` 永不打开 AOS 控制 socket。
- `commissioned:=true` 是操作者声明，不是自动安全检测；配合 `require_preflight:=true`
  时还需要一份新鲜的、全通过的预检报告。
- 软件零速是停车请求，**不是**硬件急停。live 测试必须有人握硬急停。
- 操作员网关是**独占控制权**：断连即停，不自动恢复。

### 13.2 仍未完成（必须实机）

1. **从未向 AOS 发送过速度指令** —— `Cmd=21/25` 的实际单位、符号、最低有效速度待实测。
2. 本体 500 ms 速度超时、实际停车距离、硬急停行为未实测。
3. `axis_max_x/y/yaw`（`Cmd=21` 归一化满量程）需实测标定。
4. 自车遮罩余量、停止区尺寸需按实车腿部扫掠复调。
5. PTP 同步（`rsdriver ptp=0x1`、时间差）未验证。
6. `snap` 策略、`enable_lateral` 两项默认关闭，需逐项实机接受。
7. 二维单层切片的固有盲区：负障碍、下台阶、悬空障碍、玻璃/玻璃门。
8. 目标身份跟踪：算法取目标圆内点云质心，第二个人走进该圆会被误当目标。

---

## 14. 文档索引

| 文档 | 内容 |
| --- | --- |
| [docs/M20_BLOCKERS_CLEARED.md](docs/M20_BLOCKERS_CLEARED.md) | **五个硬阻塞的逐条清除记录与验证证据** |
| [docs/M20_ACCEPTANCE_CHECKLIST.md](docs/M20_ACCEPTANCE_CHECKLIST.md) | **最小验收 checklist（GOS 可直接执行）** |
| [docs/OPERATOR.md](docs/OPERATOR.md) | 操作员网关部署、命令、Android 协议 |
| [docs/OPERATOR_SPEC.md](docs/OPERATOR_SPEC.md) | 网关接口规格 |
| [docs/LIDAR_CHOICE_ANALYSIS.md](docs/LIDAR_CHOICE_ANALYSIS.md) | 融合点云 vs 仅前雷达选型分析 |
| [docs/DOC_DRIFT_2026-09-09.md](docs/DOC_DRIFT_2026-09-09.md) | 19 篇官方文档与快照的差异比对 |
| [docs/M20_ADAPTATION.md](docs/M20_ADAPTATION.md) | 适配边界与协议推理 |
| [docs/M20_RUNBOOK.md](docs/M20_RUNBOOK.md) | 扩展运行手册与验收模板 |
| [docs/M20_DRY_RUN_2026-09-08.md](docs/M20_DRY_RUN_2026-09-08.md) | 实机 dry-run 记录 |
| [docs/M20_GUARD_CHECK_2026-09-08.md](docs/M20_GUARD_CHECK_2026-09-08.md) | 停止区只读观察 |
| [docs/M20_DEPLOYMENT_2026-09-07.md](docs/M20_DEPLOYMENT_2026-09-07.md) | GOS 部署记录 |
| [docs/M20_SOURCES.md](docs/M20_SOURCES.md) | 厂商文档证据索引 |
| [docs/DEAMON2_ORIGINAL_README.md](docs/DEAMON2_ORIGINAL_README.md) | 上游 deamon2 原说明（归档） |
| [legacy/m20_follow_control](legacy/m20_follow_control) | 早期独立三维实现（`COLCON_IGNORE` 排除） |

---

## 15. 来源与许可证

- 上游算法与 M20 适配：`6-robot/jie_deamon`、`bernardleex526/deamon2`（MIT）。
- 本仓库保留原作者信息；`LICENSE` 见根目录。
- 厂商文档（硬件/软件使用/软件开发/协议/雷达等 19 篇）的快照位于 `docs/vendor_text/`，
  差异比对结论见 [docs/DOC_DRIFT_2026-09-09.md](docs/DOC_DRIFT_2026-09-09.md)。
