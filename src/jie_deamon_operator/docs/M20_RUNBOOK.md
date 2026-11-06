# M20 Pro 运行与验收手册

2026-09-07 已在 GOS 完成源码同步、ARM64/Foxy 构建和隔离域行为测试。
2026-09-08 已在 GOS root 环境确认新鲜应用层点云和扫描约 10 Hz，完成远程 RViz 显示、
停止区检查和静止 dry-run 目标预览。最新事实与未通过的运动验收项见
[2026-09-08 dry-run 记录](M20_DRY_RUN_2026-09-08.md)。

本手册保留离线、Linux 构建和现场验收细节。它不代表机器人已完成实机跟随验收；执行现场步骤前，
仍必须遵循根目录 [README](../README.md) 的 SOP 和控制权边界。

## 1. 当前电脑可执行

进入本仓库目录后：

```powershell
python -m unittest discover -s test -v
```

测试包含真实 UDP 回环 socket，目标严格为 `127.0.0.1`，不会连接机器人。
协议/状态测试不依赖 ROS，也不需要管理员权限。

## 2. 无机器人 Linux 构建

首选 Ubuntu 20.04 + Foxy 与原厂一致；可另用 Ubuntu 22.04 + Humble 验证应用兼容性。
安装依赖需要先联网；依赖已缓存后可断网执行构建和下述演练。
当前交付的是源码与流程，不是包含全部 ROS/ARM 依赖的完全断网安装镜像。

把本仓库置于 `~/m20_ws/src/jie_deamon`。已安装相应 ROS 后执行：

```bash
source /opt/ros/foxy/setup.bash
sudo apt-get update
sudo apt-get install -y libopencv-dev python3-colcon-common-extensions \
  ros-${ROS_DISTRO}-ament-cmake-python ros-${ROS_DISTRO}-ament-cmake-gtest \
  ros-${ROS_DISTRO}-ament-lint-auto ros-${ROS_DISTRO}-ament-lint-common \
  ros-${ROS_DISTRO}-pointcloud-to-laserscan
cd ~/m20_ws
colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select jie_deamon --ctest-args -R test_m20_tracker --output-on-failure
colcon test-result --verbose
cd src/jie_deamon
python3 -m unittest discover -s test -v
python3 test/ros_m20_smoke.py
```

Humble 环境将第一行改为其 setup 路径。smoke 脚本仅适用于 Linux，自动使用 Domain 83 和
`ROS_LOCALHOST_ONLY=1`，启动完整点云转换、原算法和桥，检查前向跟随、目标丢失、近障锁定和输入中断。
CI 提供同样的 Foxy/Humble 构建与 smoke 流程；未执行前不能判为通过。

## 3. 手动合成数据演练

每个终端都 source 同一工作区，并设置：

```bash
source ~/m20_ws/install/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

终端 A：

```bash
ros2 launch jie_deamon m20.launch.py cloud_topic:=/m20/mock_points
```

终端 B：

```bash
ros2 run jie_deamon m20_mock_inputs
```

终端 C，选中前方 1.8 m 合成目标并启动跟随：

```bash
ros2 topic pub --once /robot_nexus/target geometry_msgs/msg/Point '{x: 1.8, y: 0.0, z: 0.0}'
ros2 service call /robot_nexus/set_moving std_srvs/srv/SetBool '{data: true}'
ros2 service call /m20/arm std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /m20/bridge_status
```

预期出现 `dry_run: true`、`armed: true`，前向速度约 0.3 m/s，**只发布预览，不打开 UDP socket**。
`/m20/cmd_vel_raw` 是算法意图；`/m20/cmd_vel_guarded` 才是门控后的指令，二者均不是机器人的实测速度。

逐项演练：

```bash
# 移除目标，原算法应输出零。
ros2 param set /m20_mock_inputs target false
# 恢复目标并插入近障，桥应解除使能。
ros2 param set /m20_mock_inputs target true
ros2 param set /m20_mock_inputs obstacle true
# 清除障碍仍不能自动重新使能。
ros2 param set /m20_mock_inputs obstacle false
# 人工重新使能后才可继续。
ros2 service call /m20/arm std_srvs/srv/SetBool '{data: true}'
# 主动停止。
ros2 service call /m20/arm std_srvs/srv/SetBool '{data: false}'
```

再停止终端 B，确认指令/扫描/状态过期后不再放行。不要在真实机器人 Domain 运行合成输入节点。
Web 只作为离线可视化选项：`enable_web:=true`，对应 8080/8890 端口。
上游 Web 没有鉴权且握手代码调用 shell，本交付的 M20 live launch 禁止启用它；Android 与 D1 动作始终关闭。
上游 Web 多线程退出等行为未全面重构，离线调试结束可直接停止整个 launch。

## 4. rosbag 回放

在隔离的 Domain 83 上运行，录包应包含原始 `/LIDAR/POINTS`。

```bash
ros2 launch jie_deamon m20.launch.py use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

没有 BasicStatus 时桥保持零输出，可先看 `/m20/scan` 与原算法点云匹配结果。
如需模拟使能，单独向 dry-run 专用 `/m20/mock_basic_status` 以 2 Hz 以上发布完整 JSON 状态，
可参考 `mock_inputs.py`，不要同时混入另一套合成点云。
时钟暂停或 seek 会触发过期/未来帧检查，需要恢复数据后重新 arm。
live 桥拒绝 `use_sim_time=true`。

## 5. 机器到手后的只读检查

先记录固件版本，目标最低 V1.1.7，结合厂家意见使用包含跨 ROS 监听修复的版本。
登录 GOS `user@10.21.31.104`；NOS `user@10.21.31.106` 仅作必要原厂服务检查。
AOS 不提供常规用户 SSH/VNC，不能把需要登录 AOS 的方案作为实施前提。

GOS：

```bash
source /opt/robot/scripts/setup_ros2.sh
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
ip -br address
ip route
ros2 topic list -t
ros2 topic info /LIDAR/POINTS -v
ros2 topic hz /LIDAR/POINTS
systemctl status rsdriver.service --no-pager
timedatectl
```

实际需 Domain=0，跨主机 `ROS_LOCALHOST_ONLY=0`。优先沿用厂商 setup 对 DDS 的配置，
外接标准 ROS 用 Fast DDS；不要覆盖已有 DrDDS 配置文件或复制不匹配的 RMW 库。
使用 `ros2 topic echo` 的 sensor-data/best-effort QoS 选项查看头部、字段和少量数据，避免打印整帧导致终端过载。
确认 frame、x/y/z、点云合并、频率约 10 Hz 和 header 时间戳。

NOS/GOS 检查 `systemctl cat multicast-relay.service` 与运行日志，确认实际转发主机及网卡，
再按厂商文档在**服务所在主机**为本次运行启动：

```bash
sudo systemctl start multicast-relay.service
systemctl status multicast-relay.service --no-pager
journalctl -u multicast-relay.service -n 50 --no-pager
```

是否开机自启是单独的运维决定，不是 dry-run 或现场测试的隐含步骤。

如果两台都没有该 unit，先确认固件与厂商交付；不要自行猜测 relay 命令或修改雷达目的地址。
PTP：检查 rsdriver 日志 `ptp=0x1`、时间差，确认 NOS Master/GOS Slave 同步。
使用真实回包判断 UDP 可达；`nc -zv ...30000` 默认是 TCP，不能证明 UDP 服务工作。

## 6. NOS 官方建图与 GOS 被动观察

`jie_deamon` 不负责 SLAM 或地图保存。官方建图程序只在 NOS 上运行；GOS 不启动
`m20.launch.py`，不启动桥接节点，也不发布任何速度命令。

建图前确认机器人静止站立、路线已清理、门已打开、无人员频繁穿行或雷达遮挡。NOS 上按需要启动：

```bash
# 默认名称、启动 RViz、结束后激活地图。
sudo drmap mapping

# 指定地图名称，或在不需要 RViz 时添加 -s；不自动激活地图时添加 -b。
sudo drmap mapping -n my_map -s
```

出现 `Building map` 后，操作者使用原厂遥控器按预定路线平稳行走。结束时在 NOS 执行：

```bash
sudo drmap stop_mapping
```

若只需要在 GOS 被动检查同一份融合点云的二维切片，使用下面的观察器。它只有一个
`pointcloud_to_laserscan` 进程，不包含 `robot_nexus`、`m20_bridge`、Web、Android、AOS UDP
或 `/cmd_vel` 发布者：

```bash
source /opt/robot/scripts/setup_ros2.sh
source ~/m20_ws/install/setup.bash
ros2 launch jie_deamon m20_mapping.launch.py
```

输入为融合 `/LIDAR/POINTS`，输出为本地 `/m20/mapping_scan`。不要修改
`send_separately`；独立双雷达模式会影响原厂导航、定位与充电功能。

## 7. GOS 编译与被动验证

在 GOS 用户工作区编译，不覆盖系统 `/opt/robot`。采用原厂 Foxy 环境与本地 ARM64 OpenCV/依赖。
在没有运行官方建图、导航、定位或充电任务时，才可保持 `dry_run=true` 启动跟随链路：

```bash
source /opt/robot/scripts/setup_ros2.sh
source ~/m20_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
taskset -c 4-7 ros2 launch jie_deamon m20.launch.py
```

用静态物体检查正前方点 x>0、左侧 y>0；确认无遮挡时有效帧，停止区内障碍能触发解除使能。
调试 `min_height/max_height` 前确认 z 是以机身原点而非地面为零。
配置中的身体排除区只用于原算法，桥不屏蔽自体点。不能为消除误停而盲目扩大排除区域。
确认负载下的 `/m20/scan`、raw/guarded 指令频率，测量 CPU、RSS、温度与端到端时间。

## 8. 运控接管验收顺序

以下是机器到手后执行的清单，本次没有执行任何一项实体动作。

1. 操作者准备好原厂遥控和硬急停，固定可控测试区域；确认实际腿部扫掠范围、刹停距离和近障参数。
2. 通过原厂控制端完成起立、导航使用模式和步态选择；每次模式切换会重置步态，必须重新读取 BasicStatus。
3. 在 NOS 确认原厂导航/充电任务空闲；按文档停 `planner.service`，核对是否还有其他速度控制程序或自启拉起。
   可用 `systemctl status planner.service`、`ros2 topic info /NAV_CMD -v` 辅助检查。
   保留 basic_server、rl_deploy、雷达驱动和时间同步服务，不运行另一套接管客户端。
4. 确认 MotionState=17、ControlUsageMode=1、Direction=0、Charge=0、HES=0、Sleep=0；
   采用已验证的步态，初期可用敏捷平地 12290，先核实与实际固件对应。
5. 只有这些条件已核验后，才在 GOS 重启为 live：

```bash
ros2 launch jie_deamon m20.launch.py dry_run:=false commissioned:=true
```

6. live 启动即向 AOS 发心跳和零速度，但不会自动 arm、起立、趴下、切换模式或步态。
   它也不验证 planner 已关闭；`commissioned=true` 是操作方确认已经完成接管检查，不是自动检测结果。
7. 先确认 `/m20/bridge_status` 能稳定收到完整 BasicStatus，没有协议拒绝；保持算法不使能。
8. 选定实测目标，调用跟随使能和 `/m20/arm`，验证 X/Y/Yaw 方向、零速和小速度响应。不要跳过最低速度验证。
9. 实测：停止算法、断点云、停桥进程、断网络、切回手动、触发硬急停、充电非空闲、遮挡单雷达。
   软件可处理的故障应置零并锁定；杀进程/断网依赖本体速度超时停车。
10. 完成后先 disarm，确认实测静止，再退出桥，按现场接管前记录恢复原厂 planner/使用模式。

软件中的零速度是减速停车请求，**不是**软急停。M20 软急停意味着关节断电，不能替代日常停车。
本桥没有执行软急停、站立、趴下和步态切换的自动状态机，继续使用原厂控制端完成这些操作。

## 9. 验收记录模板

| 项目 | 记录 |
| --- | --- |
| 固件、系统、ROS、DDS 版本 | 待填 |
| AOS/NOS/GOS 实际 IP；relay 所在主机及接口 | 待填 |
| PointCloud2 话题、发布者、QoS、frame、坐标与频率 | 待填 |
| PTP 状态、扫描年龄最大值 | 待填 |
| BasicStatus 原始回包及各状态位 | 待填 |
| 当前步态、最小有效速度和各轴符号 | 待填 |
| 单控制源、planner 停止及恢复记录 | 待填 |
| 动态轮廓、切片高度、停止区域、实测停车距离 | 待填 |
| 断流/断网/杀进程/急停/充电/单雷达故障结果 | 待填 |
| GOS CPU/内存/温度、扫描到发包延迟 | 待填 |

这些记录完成前，状态应保持为“离线适配完成，待编译与实机验收”，不能标为量产可用。
