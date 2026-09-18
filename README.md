# deamon2 — M20 Pro 点云跟随工程（发布快照）

> ## ⚠️ 安全声明（先读）
> - 本仓库是 **2026-09-18 的发布快照**，部署源码与测试与 GOS104 上
>   `/home/user/new_chase` 当前源码**逐字相同**（仅目录布局不同：ROS 包位于
>   `src/jie_deamon/`，运动门控位于 `motion_control/`）。
> - **真实长时间连续跟随当前不支持，也没有任何自动 live 循环**。软件允许的
>   单会话运动窗口上限是 `--motion-seconds ≤ 10` 秒（该限值由单元测试与
>   域 47 隔离测试验证）；**硬件上只实际验证过 0.5 s 与 3 s 两个运动窗口**。
>   不要用任何 live 循环代替长时间运动验收；长时间跟随需要另行设计看门狗、
>   急停链路与几何标定，并重新验收。
> - 实机运动流程是**分段的、每段需人工确认的危险操作**，不是一键脚本。
>   本 README 不提供任何自动切模式/起立/步态/一键跟随命令；文中所有
>   source/environment 命令只设置 shell 变量，不会让机器自动站立、改步态或切模式。
> - 零速补发与 exit 0 **不保证机械停止**；兜底只能靠机端看门狗与现场人工急停。
> - 几何参数（0.46/0.46/0.31/0.31、矩形 0.7、跟随距离 1.2）是**静态估计、
>   未标定**，不作为安全证明。
>
> ## 来源与上游
> - 本工程基于上游 **https://github.com/6-robot/jie_deamon** 的
>   commit `9403185c0cf99a0b415ddd4f3b4215b44dd0809c`（M20 适配均在此基线上
>   完成；上游包 `src/jie_deamon/package.xml` 保留其原始 MIT 许可证声明，
>   本仓库不新增或伪造许可证）。
> - **文档层级**：本根 README 是唯一权威操作文档。子目录中随源码保留的
>   `motion_control/*.md`、`src/jie_deamon/docs/NEW_CHASE.md` 等是**历史阶段
>   记录**：其中上游 D1 或 `start.launch` 的旧指令不代表本工程 M20 的推荐
>   操作，**勿照旧文档驱动 M20**。

## 目录导航

- [0. 机器与终端约定](#0-机器与终端约定)
- [1. 环境铁律与干净终端模板](#1-环境铁律与干净终端模板)
- [2. 新 clone 用户注意（不要覆盖已有部署）](#2-新-clone-用户注意不要覆盖已有部署)
- [3. 开机前检查：现场服务状态](#3-开机前检查现场服务状态)
- [4. 编译（GOS104）](#4-编译gos104)
- [5. 启动预览链路：两种方式二选一](#5-启动预览链路两种方式二选一)
- [6. 选点与预览状态（Web）](#6-选点与预览状态web)
- [7. 真实运动流程（危险：分段人工确认）](#7-真实运动流程危险分段人工确认)
- [8. 长时间计划（当前只支持干跑）](#8-长时间计划当前只支持干跑)
- [9. 测试（发布前已在源机器全部通过）](#9-测试发布前已在源机器全部通过)
- [10. 仓库内容速览](#10-仓库内容速览)

```
.
├── config/diagnostic_udp_only.xml   # 批准的 UDP-only Fast-DDS 诊断配置（无 SHM）
├── docs/VALIDATION.md               # 已验收历史摘要（安全公开版）
├── motion_control/                  # 运动放行门控 + 有界 dryrun/live 运行入口（py + md）
├── src/jie_deamon/                  # ROS 2 包（上游 jie_deamon + M20 适配）
├── test/                            # 全部现有测试 + C++ 诊断探针
└── tools/                           # drdds_cloud_export 源码 + 编译脚本
```

## 0. 机器与终端约定

本文所有终端均按机器标注，使用 `user@IP` 显式地址，不假设 SSH 别名：

| 标注 | 机器 | IP | 角色 |
|---|---|---|---|
| GOS104 | 主控机 | 10.21.31.104 | 跑本工程所有 ROS/SDK 进程、Web（8080/8890） |
| NOS106 | 导航机 | 10.21.31.106 | 原厂 planner 服务所在 |
| AOS103 | 运动机 | 10.21.31.103 | 遥测 UDP 上报方（10.21.31.103:30000） |

环境：**ARM64 + Ubuntu 20.04 + ROS 2 Foxy**。`pointcloud_to_laserscan` 与
机器上**已安装的厂商 DrDDS SDK**（libdrdds / FastDDS 2.14）在 GOS104 上
已就绪，先用下文检查命令确认存在，**不要未经核实就用 apt 安装任何厂商包**
（本文不指认 SDK 的制造商归属，以机器实际安装为准）。

```bash
# 在 GOS104 上检查依赖是否就绪（只检查，不安装）：
ls /opt/ros/foxy/setup.bash
ls /opt/ros/foxy/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node
ls /usr/local/lib/libdrdds.so*
ls /usr/local/include/dridl 2>/dev/null || ls /usr/local/include
g++ --version | head -1
python3 --version
```

## 1. 环境铁律与干净终端模板

1. **绝对禁止 `source /opt/robot/scripts/setup_ros2.sh`**——它会改写工厂
   DDS 配置，影响整机其他服务。
2. 所有 ROS 终端一律用 `env -i` + `bash --noprofile --norc` 起干净环境，
   只 source 两样：`/opt/ros/foxy/setup.bash` 和
   `/home/user/new_chase/install/setup.bash`（编译之后才有）。
3. 变量五件套，**每个 ROS 终端显式设置**（下文所有块均已内嵌）：
   - `ROS_DOMAIN_ID=0`
   - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
   - `FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml`
     （UDP-only、无 SHM；`motion_control.runtime` 会结构校验此文件，SHM 配置直接拒绝）
   - `ROS_LOCALHOST_ONLY=0`
   - `PYTHONDONTWRITEBYTECODE=1`（配合 `python3 -B`，不产生 `__pycache__`）
4. **用户主路径固定为 `/home/user/new_chase`**（不是任何 `m20_chase`）。
   下文所有命令都按此路径书写，直接可复制。
5. 每个块内 `set -e`：**source 失败立即中断**，绝不"source 失败继续执行后面的命令"。

### 1.1 干净诊断终端模板（交互式；后文"诊断终端"均指它）

第 7 节的 `ros2 topic` / `ros2 service` 诊断与 arm 命令**必须在这个终端执行**。

第一步——在 GOS104 打开干净环境交互 shell（注意**没有 `-c`**，进入的是交互终端）：

```bash
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc
```

第二行起——在刚打开的干净 shell 内逐行执行（`set -e` 只保护两行 source
失败立即停下；环境就绪后 `set +e`，见下）：

```bash
set -e
source /opt/ros/foxy/setup.bash
source /home/user/new_chase/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
export PYTHONDONTWRITEBYTECODE=1
cd /home/user/new_chase
set +e   # 环境就绪后恢复：诊断命令返回非0不会退出本终端
echo "干净诊断终端就绪: domain=0 rmw=fastrtps profile=UDP-only"
```

- `set -e` 期间 source 失败（如还没编译过、`install/setup.bash` 不存在）
  会立即停下——这是预期保护，不带污染环境继续；先完成第 4 节编译。
- `set +e` 之后，诊断命令可正常返回非 0，需**人工判读**：例如
  `ros2 topic info` 对不存在的话题返回 1——**话题不存在不等于证明独占
  或无他方**；独占核查还须结合 runtime ownership 诊断与现场人工确认。
- 该终端只是设置 shell 环境变量，**不会让机器站立、改步态或切模式**。

## 2. 新 clone 用户注意（不要覆盖已有部署）

- 如果你是新机器新 clone：**只在目标路径不存在时** clone；如果
  `/home/user/new_chase` 已存在且可能是脏工作区，**先停下来人工确认**，
  绝不覆盖：
  ```bash
  # 新机器：
  if [ -e /home/user/new_chase ]; then
      echo "路径已存在，人工确认后再决定，不要覆盖"; exit 1
  fi
  git clone https://github.com/bernardleex526/deamon2.git /home/user/new_chase
  ```
- **GOS104 上的现有部署已经在 `/home/user/new_chase` 可直接使用**：现场
  transient 服务（见第 3 节）正按该路径运行，不需要为了"更新"而动它。
- 不要在嵌套的 `src/jie_deamon` 里对它的旧 origin 执行 `git pull`——嵌套
  仓库会与外层工程状态脱节。
- 将来的版本更新步骤（人工、分步、先备份，**禁止 `rm -rf` 硬清理**）：
  ```bash
  # 1) 安全停止相关进程（见第 3/7 节；确认无 live、已解除 arm）
  # 2) 备份当前工程：
  cp -a /home/user/new_chase /home/user/new_chase.bak-$(date +%Y%m%d-%H%M%S)
  # 3) 人工迁移：逐目录对比新快照与当前目录，只覆盖确认要更新的文件；
  #    迁移 config/ 与几何参数时逐项人工核对
  # 4) 重新编译（第 4 节）并用第 9 节测试验证后再恢复服务
  ```

## 3. 开机前检查：现场服务状态

发布快照时刻（2026-09-18），GOS104 上有**三个 systemd transient 服务正在
运行**，另有一个 nav 预览服务**处于停止状态**（临时服务重启后消失，不随
开机自启）：

| 服务 | 快照时刻状态 | 内容 |
|---|---|---|
| `new-chase-cloud-export.service` | active/running | root 运行 `drdds_cloud_export --continuous`（SDK 点云 → `/new_chase/points`） |
| `new-chase-scan.service` | active/running | user 运行 pointcloud_to_laserscan（`/new_chase/points` → `/new_chase/scan`） |
| `new-chase-preview.service` | active/running | user 运行 robot_nexus（m20_preview 显式参数，Web 8080/8890） |
| `new-chase-nav-preview.service` | **inactive / PID 0** | nav_cmd_preview 预览发布器（发布时已停止；状态以现场 `systemctl show` 核对为准，**不要声称它不存在，也不要顺手启动它**） |

动手前先检查（只查询，不会启动任何服务；对照第 5 节决定"沿用现服务"还是
"停掉后手工多终端"）：

```bash
ssh user@10.21.31.104
systemctl show new-chase-cloud-export.service new-chase-scan.service \
  new-chase-preview.service new-chase-nav-preview.service \
  -p LoadState -p ActiveState -p SubState -p MainPID
ss -tlnp | grep -E ':8080|:8890'
```

## 4. 编译（GOS104）

```bash
ssh user@10.21.31.104
cd /home/user/new_chase

# (1) 编译 SDK 点云导出器：普通 shell 即可，无需 ROS 环境
#     （只链接厂商 SDK 栈，不链接 ROS、不 touch /opt/ros；只编译不运行）
bash tools/build_cloud_export.sh

# (2) 编译 ROS 包：干净环境完整命令（source 失败即中断；关闭测试目标）
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    cd /home/user/new_chase
    colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=OFF'
```

- `build_cloud_export.sh` 只链接厂商 SDK 栈（libdrdds / libfastrtps /
  libfastcdr / pthread），从不运行产物。
- colcon 关闭 `BUILD_TESTING` 是部署路径的既定做法；本仓库**未经完整 lint
  验证**（ament_lint 未跑），知悉后再用。

## 5. 启动预览链路：两种方式二选一

### 方式 A：沿用现场现成服务（推荐，GOS104 上已运行）

直接使用第 3 节确认过的三个运行中的服务，无需任何启动动作。Web 地址
`http://10.21.31.104:8080`。要重启某个服务用
`sudo systemctl restart new-chase-scan.service`（临时服务，重启机器后消失，
不会自启用）。

### 方式 B：先停掉三个运行中的服务，再手工多终端启动

**仅当确认当前无 live 运动会话、arm 已解除**（见第 7 节；预览链路本身
不涉及运动，但统一在无 live 状态下操作链路）后才停止：

```bash
sudo systemctl stop new-chase-cloud-export.service new-chase-scan.service new-chase-preview.service
systemctl is-active new-chase-cloud-export.service new-chase-scan.service new-chase-preview.service   # 三行都应 inactive
ss -tlnp | grep -E ':8080|:8890'    # 应无输出
```

- `new-chase-nav-preview.service` 不在上面的停止命令里：如它当时在运行，
  人工决定是否单独 `sudo systemctl stop new-chase-nav-preview.service`，
  不要由脚本顺带处理，更不要启动它。
- 若现场已有 live runtime 进程，**禁止**执行本节停止/启动操作。

然后开**三个终端**（显式命令，不用默认 run_preview.sh / launch，原因见
本节末尾的缺陷说明）：

**终端 1（root，SDK 点云导出器，无 ROS 环境）：**
```bash
ssh user@10.21.31.104
sudo env -i HOME=/root USER=root \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /home/user/new_chase/bin/drdds_cloud_export --continuous
# 短验收可用 --seconds 25 替代 --continuous；密码现场人工输入，不写进任何文件
```

**终端 2（普通 user，点云→LaserScan 转换器）：**
```bash
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    exec ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
      --ros-args \
      -r cloud_in:=/new_chase/points \
      -r scan:=/new_chase/scan \
      -p min_height:=-0.05 \
      -p max_height:=0.50 \
      -p angle_min:=-3.141592653589793 \
      -p angle_max:=3.141592653589793 \
      -p angle_increment:=0.00872665 \
      -p range_min:=0.10 \
      -p range_max:=20.0'
# 注意：target_frame 保持默认空（沿用点云自身 frame，不建重复 TF 链），
# 这是已实测验证的取法，不要额外添加 target_frame 参数。
```

**终端 3（普通 user，robot_nexus，显式全参数含 web_root）：**
```bash
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    source /home/user/new_chase/install/setup.bash
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    exec ros2 run jie_deamon robot_nexus --ros-args \
      -p m20_preview:=true \
      -p active:=true \
      -p enable_web:=true \
      -p enable_opencv:=false \
      -p web_root:=/home/user/new_chase/install/jie_deamon/share/jie_deamon/web \
      -p scan_topic:=/new_chase/scan \
      -p raw_topic:=/new_chase/cmd_vel_raw \
      -p expected_scan_frame:=lidar_link \
      -p rotation_angle:=0.0 \
      -p follow_distance:=1.2 \
      -p rectangle_width:=0.7 \
      -p robot_frame_front:=0.46 \
      -p robot_frame_back:=0.46 \
      -p robot_frame_left:=0.31 \
      -p robot_frame_right:=0.31'
```

- NavCmd 预览发布器（`nav_cmd_preview.py`）**不要在本流程中启动**；它只
  发布 `/new_chase/nav_cmd_preview` 观测话题，不参与本链路，更不发
  `/NAV_CMD`。它当前是否存在/运行以第 3 节 `systemctl show` 核对为准。
- 各终端停止：`Ctrl-C` 正常退出，不 pkill、不清理 SHM。

### ⚠️ 为什么不用默认入口（`run_preview.sh` / `new_chase_preview.launch.py`）

`launch/new_chase_preview.launch.py` 有一个**已知的、未修的缺陷**：它在
第 51 行计算了 `web_root`（包 share 下的 web 目录），但 robot_nexus 的
参数字典**没有把 `web_root` 传进节点**，导致默认 launch 起的 Web 首页
404。本快照**不改源码、不顺手修 launch**，因此推荐入口就是本节的
显式命令（robot_nexus 显式传了 `web_root`）。`run_preview.sh` 与默认
launch 保留在源码树里仅供追溯，**不要作为推荐入口使用**。

## 6. 选点与预览状态（Web）

- 网页：`http://10.21.31.104:8080`（WebSocket 8890 是 **TCP**，与点云
  UDP export 无关，别混淆端口）。
- **选点首选网页上对最新扫描回波双击**，坐标来自当前帧实际目标；不要按
  硬编码坐标（如固定 1.7 m）盲选。
- 如需 API 选点，坐标同样必须取自当前帧实际目标，用交互输入避免盲复制
  （JSON 必须带 `Content-Type: application/json`；非法坐标服务端返回
  HTTP 400 拒绝）：
  ```bash
  # x/y 按网页/最新扫描帧上的实际目标人工输入：
  read -r -p "target x(m): " x
  read -r -p "target y(m): " y
  curl -sS -X POST http://10.21.31.104:8080/api/set_target \
    -H 'Content-Type: application/json' -d "{\"x\": ${x}, \"y\": ${y}}"
  ```
- `/api/set_moving {"enabled": true}` 只在**确认无 live 运动会话、已完成
  上面选点**之后执行，并先核对状态 flags：
  ```bash
  curl -sS http://10.21.31.104:8080/api/status
  curl -sS -X POST http://10.21.31.104:8080/api/set_moving \
    -H 'Content-Type: application/json' -d '{"enabled": true}'
  ```
- runtime 侧认可的四个标志是 `is_active` / `is_selected` /
  `is_tracking_valid` / `is_moving_enabled` **同时为 true**
  （`GET /api/status` 可读）。
- ⚠️ **开启 Web 使能前必须：无 live 运动会话、nav_cmd_preview 预览发布器
  已停止**（以第 3 节 `systemctl show` 核对为准）。
  `m20_preview=true` 只是禁掉上游旧 D1 接口（`/cmd_vel`、`/d1_cmd` 等），
  **不阻止**独立的 runtime live 链路。
- 区分两种"输出"：
  - `/new_chase/cmd_vel_raw`（`geometry_msgs/Twist`）= 算法预览 raw 输出。
    **底盘不直接消费 Twist**：live 会话中本工程 runtime Controller 是它的
    消费者，经运动门控转成 `/NAV_CMD` 才是真实执行通道；
  - `/NAV_CMD`（`drdds/NavCmd`）= **真实执行**通道，只有第 7 节的
    runtime 进程（经人工授权）才允许发。

## 7. 真实运动流程（危险：分段人工确认）

> 本节每个勾选项都需要现场人工完成并确认。任何一步不符都停在原处。

### 7.1 分段前置确认

- [ ] 现场有人手够得着急停；机器狗状态确认：
      `HES=0、Charge=0、Sleep=0、MotionState=17、Gait=4097、
      ControlUsageMode=1、Direction=0`（运动模式、basic gait 4097）。
      **不自动切模式、不自动起立、不自动切步态**——这些是人工步骤。
- [ ] NOS106 上确认无导航任务、急停就绪后，停止 planner（**先人工确认，
      再执行**）：
      ```bash
      ssh user@10.21.31.106
      sudo systemctl stop planner.service global_planner.service
      systemctl status planner.service global_planner.service --no-pager   # 均 inactive、PID 0
      ```
      charge_manager 还需**人工确认空闲**（`Charge=0` 是必要不充分条件）。
- [ ] GOS104 上确认 `/NAV_CMD` 没有其他 publisher（ROS 图探测**不证明
      所有权**，逐个端点人工核对）。**必须在第 1.1 节的干净诊断终端执行**：
      ```bash
      ros2 topic info /NAV_CMD -v
      ros2 topic info /new_chase/cmd_vel_raw -v   # raw publisher 应唯一（现场 robot_nexus）
      ```
- [ ] 近障输入健康（同一诊断终端）：scan 有效且 `clear`、约 10 Hz：
      ```bash
      ros2 topic hz /new_chase/scan     # ≈10 Hz
      ```

### 7.2 dryrun 演练（无 NAV 输出，安全）

```bash
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export PYTHONPATH=/home/user/new_chase:$PYTHONPATH
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    cd /home/user/new_chase
    exec python3 -B -m motion_control.runtime --seconds 20'
```

- dryrun（默认模式）绝不 arm、绝无 `/NAV_CMD` 输出；观察日志中四个输入
  （status / scan / web flags / raw）是否有效、阻断原因是否**只有
  disarmed** 一项。首次建议先 `--seconds 10` 再 `--seconds 20`。
- dryrun 的 `--seconds` 有限界 10..120。
- **网络边界准确表述**：遥测协议内唯一的出站帧是心跳（Type=100/Cmd=100，
  只读订阅注册用）；runtime 进程本身还有 HTTP GET（127.0.0.1:8080 状态
  读取）、ROS 诊断发布与 DDS 发现等常规通信。准确说法是 **dryrun 不发送
  任何控制/速度指令**，不是"网络完全只读、没有其它包"。

### 7.3 live 短窗口（现场人工分段授权）

> 三旗 `--live --commissioned --exclusive-control-confirmed` 只能由现场
> 前置检查全部人工通过后设置；它们不是自动认证，是从"我已人工确认"翻译
> 成的命令行事实。失败不要循环重试。

**终端 A（runtime，GOS104）：**
```bash
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export PYTHONPATH=/home/user/new_chase:$PYTHONPATH
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    cd /home/user/new_chase
    exec python3 -B -m motion_control.runtime --live --commissioned \
      --exclusive-control-confirmed --motion-seconds 0.5 --seconds 30'
```

**终端 B（arm 请求，GOS104，另开一个终端；可用第 1.1 节模板或本完整块）。**
执行前必须逐项人工确认：runtime 诊断显示**唯一阻断原因 disarmed**、四个
输入（status/scan/web flags/raw）**新鲜有效**、`/new_chase/cmd_vel_raw`
publisher **唯一**、`/NAV_CMD` **无他方 publisher**。**Web 使能的时序**：
上一轮结束已关闭 Web 使能并确认无旧 live；本轮在**无 live 时**完成选点并
`set_moving true`，dryrun 验证全部输入，然后启动本 live 进程——**arm 前
Web 四 flags（is_active/is_selected/is_tracking_valid/is_moving_enabled）
必须仍为全 true**，否则 arm 必被门控拒绝。本轮 Web 使能已在无 live 时
开启；**关 Web 是结束操作（见 7.4），不是 arm 前置条件**。绝不在活跃
live 会话中重新选点或恢复使能来绕门控。确认后执行：

```bash
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    timeout --signal=INT 3s ros2 service call /new_chase/arm std_srvs/srv/SetBool "{data: true}"'
```

- arm 命令只是**请求解锁已授权 runtime 进程的输出**；它不会让机器自动
  站立、改步态或切模式（那些全部是 7.1 的人工步骤）。
- `timeout --signal=INT 3s` 防无限等待（runtime 已退出后服务调用会一直
  阻塞）；**退出码 124 = 调用被超时中断，不是成功，绝不能当作已 arm/已停**。
- 成功响应**只接受一次**；任何失败（含 124）**不循环重试**，停在原处人工分析。

规则（全部来自源码合同，逐条有效）：

- **每个进程只接受一次成功 arm**；之后 disarm / 到期 / 任何失效都锁死到
  进程结束，**不可重新 arm**，失败**不循环重试**。
- `--motion-seconds` 有效域 `(0, 10]` 秒，这是**软件允许的单会话上限**
  （由单元测试与域 47 隔离测试验证），**不是硬件验收结论**；硬件只实际
  验证过 0.5 s 与 3 s。**不提供任何延长、绕过或自动 re-arm 循环**。
- `--seconds`（20..120）是进程总时长，**不是运动时长**。
- 先用 `--motion-seconds 0.5` 验证实际停止，再进行一次**新的手工授权**
  （新进程）跑 `--motion-seconds 3`；软件上限 10 s 未做硬件验证，不宣称。
- **关停顺序**见第 7.4 节完整停止命令块：有异常优先 hardware estop。

### 7.4 完整停止命令块（人工逐条执行，不是一键脚本）

```bash
# (1) disarm：GOS104，runtime 进程仍在运行时执行（干净环境 + 3s 超时防无限等待）：
ssh user@10.21.31.104
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    timeout --signal=INT 3s ros2 service call /new_chase/arm std_srvs/srv/SetBool "{data: false}"'
# 退出码 124 = 调用被超时中断：绝不能当作已解除 arm；改用现场急停并人工核查。

# (2) Web 使能关闭（GOS104 或任意可达主机；JSON 需带 Content-Type）：
curl -sS -X POST http://10.21.31.104:8080/api/set_moving \
  -H 'Content-Type: application/json' -d '{"enabled": false}'

# (3) 核对状态（is_moving_enabled 必须 false；is_active/is_selected/
#     is_tracking_valid 可能保持 true——保留目标不等于运动授权）：
curl -sS http://10.21.31.104:8080/api/status

# (4) runtime 终端 Ctrl-C 正常退出；退出后在第 1.1 节干净诊断终端核对
#     /NAV_CMD 已无 publisher：
ros2 topic info /NAV_CMD -v

# (5) 现场人工确认机器实际静止（眼睛看 + 手急停在手边）。
```

> **零速补发、exit 0、(1)-(4) 任何命令的成功返回都不保证机械已停止。**
> 任何异常（超时 124、发送错误、行为异常）直接**现场急停**，不要继续命令。

### 7.5 结束后

- 安全停止后**保持 planner 停止**，不自动恢复。若要恢复原厂导航任务，
  人工步骤（先确认 runtime 已退出、`/NAV_CMD` 无 publisher）：
  ```bash
  ssh user@10.21.31.106
  sudo systemctl start planner.service global_planner.service
  systemctl status planner.service global_planner.service --no-pager   # active
  ```

## 8. 长时间计划（当前只支持干跑）

**只能用连续 dryrun 做长时稳定性观察**，例如 30 分钟 ≈ 15 次 120 s
dryrun。**30 分钟只是建议节奏，本快照发布前未实际跑过该循环**；不强制
连续等待，每轮之间按回车人工放行。**如果现场已有 live runtime 进程，
禁止运行本循环**（避免同名/同域混淆；本循环是普通用户 shell，不注册任何
服务）。loop 只做 dryrun，sender 禁用，绝无 NAV 输出；`exit 0` 只表示
"有界运行正常结束"，**不是数据 healthy 的证明**，每轮检查仍须人工做：

```bash
# 在 GOS104 普通 user 终端执行（无需 ROS 环境；循环体内部自带干净环境）。
# 失败即 break；每轮人工检查后按回车才继续；Ctrl-C 随时退出。
cd /home/user/new_chase
set -o pipefail
LOGDIR="/home/user/new_chase/log/dryrun-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOGDIR"
for i in $(seq 1 15); do
  echo "=== round $i start $(date -Is) ===" | tee -a "$LOGDIR/rounds.log"
  if ! env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      bash --noprofile --norc -c '
        set -e
        source /opt/ros/foxy/setup.bash
        export PYTHONPATH=/home/user/new_chase:$PYTHONPATH
        export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
        export PYTHONDONTWRITEBYTECODE=1
        cd /home/user/new_chase
        python3 -B -m motion_control.runtime --seconds 120' \
      2>&1 | tee -a "$LOGDIR/round-$i.log"; then
    echo "=== round $i 运行异常（含管道错误），停止循环，人工检查 ===" | tee -a "$LOGDIR/rounds.log"
    break
  fi
  # ---- 每轮人工检查（不自动判定 healthy，逐项看，异常就 break/Ctrl-C）----
  #  1) "$LOGDIR/round-$i.log" 中 sender 调用计数 calls==0
  #  2) 在第 1.1 节诊断终端：ros2 topic hz /new_chase/scan 仍 ≈10 Hz
  #  3) 磁盘 df -h /home/user/new_chase/log 与温度
  #     cat /sys/class/thermal/thermal_zone*/temp
  #  4) 停止区/stopzone 判定正常
  echo "--- round $i 检查完毕：按回车继续下一轮，Ctrl-C 结束全部 ---"
  read -r
done
```

> ## ⛔ 红线
> - **不能用任何 live 循环代替连续长时间运动验收**（本工程也没有任何
>   自动 live 循环入口）。
> - 真实长时间跟随当前不支持：需要另行设计看门狗、急停链路、几何标定并
>   重新验收。软件允许的单会话上限是 `--motion-seconds ≤ 10`（仅软件级
>   验证），**硬件只验证过 0.5 s 与 3 s**。

## 9. 测试（发布前已在源机器全部通过）

```bash
ssh user@10.21.31.104
cd /home/user/new_chase

# 单元测试（131 用例，全部通过无 skip）：
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export PYTHONPATH=/home/user/new_chase:$PYTHONPATH
    export PYTHONDONTWRITEBYTECODE=1
    cd /home/user/new_chase/test
    python3 -B -m unittest test_motion_guard test_nav_sender test_follow_runtime -v'

# 隔离 ROS 集成测试（11 用例，域 47，真实 DDS 但与现场隔离）：
env -i HOME=/home/user USER=user LOGNAME=user SHELL=/bin/bash TERM=xterm LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/setup.bash
    export PYTHONPATH=/home/user/new_chase:$PYTHONPATH
    export ROS_DOMAIN_ID=47 ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml
    export PYTHONDONTWRITEBYTECODE=1
    cd /home/user/new_chase/test
    python3 -B -m unittest test_follow_runtime_ros -v'
```

- **严禁在 `ROS_DOMAIN_ID=0` 上运行隔离集成测试**（域 0 是现场域）。
- `test_preview_ros` 会占用 8080/8890：**现场服务开着时不能跑**，也
  **不要**放进默认 all-discover（`python3 -m unittest discover` 会触发它）。
- `test_follow_runtime_ros` 内含对域与批准 profile 的硬校验，配置不对会
  直接失败，这是设计行为。

## 10. 仓库内容速览

| 路径 | 内容 |
|---|---|
| `motion_control/` | MotionGuard 门控核心、遥测/输入/发送边界、有界 dryrun/live 运行入口（README/RUNTIME/NAV_SENDER 文档齐全；**历史阶段记录，不代表本 README 的推荐操作**） |
| `src/jie_deamon/` | ROS 2 包：robot_nexus（含 M20 预览参数）、lidar_tracker 算法、Web（8080/8890）、launch、nav_cmd_preview |
| `test/` | 门控/发送/运行时单测、隔离 ROS 集成测试、C++ 诊断探针 `drdds_cloud_probe.cpp`、现场话题探针 `probe_factory_topics.py` |
| `tools/` | `drdds_cloud_export.cpp`（SDK 点云 → `/new_chase/points`，只订阅 `/LIDAR/POINTS`，绝不发任何控制帧）+ 编译脚本 |
| `config/diagnostic_udp_only.xml` | 批准的 UDP-only Fast DDS 配置（无 SHM，白名单 127.0.0.1 + 10.21.31.104） |

不在本仓库中的内容（有意排除）：evidence 原始日志、文档抓取、厂商 SDK
二进制、bin/build/install/log 产物、任何密码或 token。
