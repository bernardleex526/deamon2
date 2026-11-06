# M20 Pro 硬阻塞清除记录

日期：2026-09-09。基线：`bernardleex526/deamon2` main @ `6cf045b6`（2026-09-08）。
本文只记录**代码与配置层面的改动与验证证据**；实机运动验收仍未完成，见
[最小验收 checklist](M20_ACCEPTANCE_CHECKLIST.md)。

## 0. 一句话结论

5 个硬阻塞在**代码层面**全部清除，并在本机 WSL（Ubuntu 22.04 + ROS 2 Humble）用
12 个「可移动模拟人」场景验证了算法与门控输出（12/12 通过，见 §7）。
**但这不等于实机验收**：真机上仍未发送过一条速度指令，`commissioned:=true` 从未执行。

## 1. 阻塞一：使用模式冲突（Cmd=25 仅导航模式）

**原因**：官方《运动控制（basic_server 协议）》4.5 明确 `Cmd=25` 仅导航模式可用，
`Cmd=21` 仅常规/辅助模式可用；而实测 AOS 状态为 `ControlUsageMode=0`（常规模式），
旧门控硬编码要求 `==1`，因此永远不会放行。旧仓库也没有实现 `Type=1101 Cmd=5` 切模式。

**改法（不再要求切模式）**：门控改为**按实时模式选择子命令**。

| `control_profile` | 使用模式 | 下发命令 | 单位 |
| --- | --- | --- | --- |
| `auto`（默认） | 1 导航 | `Type=2 Cmd=25` | m/s、rad/s |
| `auto`（默认） | 0 常规 | `Type=2 Cmd=21` | 归一化 [-1,1] |
| `auto`（默认） | 2 辅助 | 拒绝（需 `allow_auxiliary_mode:=true`） | — |
| `si` | 仅 1 | `Cmd=25` | m/s |
| `normalized` | 仅 0 | `Cmd=21` | 归一化 |

- `m20_adapter/core.py::select_command()` 是唯一判定入口，模式与档位不匹配即拒绝放行，
  并在 `/m20/bridge_status` 的 `reason` 里写清原因。
- 归一化换算：`X / axis_max_x` 等，`axis_max_*` 默认 `2.0 / 1.0 / 2.0`（官方文档中
  各步态的最大值），**必须实机核对**，因为官方只写「相对于该轴最大速度」。
- 模式变化时桥会打印 `control command changed: 25 -> 21 (usage mode 0)`。
- **门控拒绝在速度流未就绪时 arm**：`arm()` 现在要求 `/m20/cmd_vel_raw` 在
  `command_timeout`（0.3 s）内有过有效指令，否则返回失败并给出
  `no fresh command stream`。这消除了「先 arm、0.3 s 后立刻因无指令自动解除」的
  假动作，也把「算法链路是否真的在跑」变成 arm 的前置条件。
- **仍然不自动切模式**：切模式会重置步态，且导航模式可能拉起原厂 planner，属安全相关动作，
  保持由操作者用原厂控制端完成（见 [验收 checklist §3](M20_ACCEPTANCE_CHECKLIST.md)）。

## 2. 阻塞二：步态最小速度与限幅矛盾

**原因**：官方 V1.1.7+ 步态速度表是**分段**的（零点附近不可用）。旧配置
`max_vy=0.3` 小于基础步态 Y 轴下限 `0.35`，导致 Y 轴被静默清零；换敏捷步态时
0.3 的横移又会被原样放行。

**改法**：把官方整张表写进 `m20_adapter/core.py::GAIT_TABLE`，门控按**实时步态**做三段处理：

| 条件 | `gait_min_policy=zero`（默认，fail-closed） | `gait_min_policy=snap`（需实机接受） |
| --- | --- | --- |
| `\|v\| < min_request`（默认 0.02） | 0 | 0 |
| `min_request ≤ \|v\| < 步态下限` | 0 | 抬到步态下限 |
| `步态下限 ≤ \|v\| ≤ min(限幅, 步态上限)` | 原值 | 原值 |
| `步态下限 > min(限幅, 步态上限)` | 0，并写 `axis_notes: "<轴> axis unusable"` | 同左 |

`GAIT_TABLE`（与官方表逐条一致，`test_gait_table_matches_vendor_ranges` 固化）：

| 步态 | X (m/s) | Y (m/s) | Yaw (rad/s) |
| --- | --- | --- | --- |
| 标准-基础 `0x1001`/4097 | [0.20, 2.0] | [0.35, 1.0] | [0.50, 2.0] |
| 标准-楼梯 `0x1003`/4099 | [0.15, 2.0] | [0.30, 1.0] | [0.40, 2.0] |
| 敏捷-平地 `0x3002`/12290 | [0.15, 2.0] | [0.25, 1.0] | [0.35, 1.5] |
| 敏捷-楼梯 `0x3003`/12291 | [0.15, 2.0] | [0.30, 1.0] | [0.40, 2.0] |

副作用（必须知道）：`zero` 策略下，误差小于「步态下限/比例系数」时机器人**完全不动**，
一超过就以步态下限起步（`LINEAR_SCALE_FACTOR=0.5`、下限 0.15 → 误差约 0.3 m 时突然
从 0 跳到 0.15 m/s）。要平滑起步就用 `snap`，但必须先在实机接受最小速度响应。

## 3. 阻塞三：横向速度算法缺陷

**原因**：D1 原式 `lateral_error = -(left_y_min + right_y_min)`，左右边界初值是
`±corridor_width/2`，**只观测到单侧墙**时该式会输出约 ±0.7 m/s，再乘
`LINEAR_Y_SCALE_FACTOR=1.0`、上限 `MAX_LINEAR_SPEED=1.0`。这正是 2026-09-08
dry-run 里 `y=+0.50 m/s` 的来源。

**改法**（`include/lidar_tracker.hpp`、`include/common_types.hpp`、`src/robot_nexus.cpp`）：

1. 记录 `left_seen` / `right_seen`；**只有双侧都真实观测到**才计算居中量。
2. 走廊宽度必须落在 `[LATERAL_MIN_CORRIDOR=0.2, LATERAL_MAX_CORRIDOR=3.0]` 才可信。
3. 改用走廊中心偏移 `0.5*(left+right)` × `lateral_gain`（默认 0.5）× `lateral_sign`，
   再按 `lateral_max`（默认 0.3）限幅、`lateral_deadband`（默认 0.05）设死区。
4. `enable_lateral` **默认 false**：M20 配置直接关闭横移；开启需先在实机验证符号与幅值。
5. 横向 APF 排斥力同样只在 `enable_lateral=true` 时生效——默认「停车而不是侧移」。
6. 桥侧另有 `axis_enable_y=false` 第二道闸，以及 `allow_reverse=false`（跟随不后退，
   目标太近就停）。

`test_m20_tracker.cpp::LateralCenteringNeedsBothWallsAndStaysBounded` 固化了
「单侧墙必须为 0」「双侧墙有界」两条。

## 4. 阻塞四：自体点导致无法 arm

**原因**：桥刻意不做自体点遮罩，实测 15 s / 150 帧中 81 帧命中停止区，
最近回波 0.45 m 位于「后左」（x=-0.367, y=+0.257），落在机身包络（半宽 0.253）之外
约 4 mm，因此旧遮罩完全无效。

**改法**（`m20_adapter/core.py::inspect_scan_report`）：

1. 自车遮罩扩到「机身框 + 每侧独立余量」：
   `front 0.41+0.05=0.46`、`back 0.41+0.14=0.55`、`left/right 0.253+0.07=0.323`。
   前向余量故意留小，保证前方停止区仍然够宽；后/侧余量覆盖腿部扫掠。
   遮罩内回波计入 `self_hits`，**不再阻断放行**。
2. 新增**前向守卫扇区** `guard_sector_min/max = ±100°`：只在扇区内判停止区。
   实测那条 159° 的后左回波直接被排除（后向倒车默认关闭，见 §3）。
3. 新增 `min_sector_bins=20`：扇区内有效回波太少就判 `invalid`（**fail-closed**）。
   这条同时把「前雷达失效但融合话题仍存活」变成可检测的失效（见
   [雷达选型分析](LIDAR_CHOICE_ANALYSIS.md) §5）。
4. 新增 `stop_hits_required`（默认 1，可设 2~3 抑制单帧噪声）、`scan_min_range=0.05`。
5. `/m20/scan_report` 新增话题，输出 `self_hits / stop_hits / sector_bins / closest_*`，
   便于现场判断「到底是自车回波还是真障碍」。

## 5. 阻塞五：root / 组播 relay 运维依赖

**原因**：官方《软件开发指南》2.1 原文「**/LIDAR/POINTS 话题必须要 su 获取管理员权限
之后才能获取到数据**」，且「被限制为仅机器人主机传输」；`multicast-relay.service`
**出厂默认关闭**（V1.1.7 起支持），重启后没点云。

**改法**：新增 `scripts/m20_preflight.py`（只读、有界）+ 启动闸门
`m20_adapter/preflight.py`。

- 检查项：`root`、`multicast_relay`（active+enabled）、`planner_idle`、
  `time_sync`（info）、`lidar_cloud`（新鲜度/点数/字段）、`scan_topic`、
  `no_nav_cmd_publisher`、`aos_link`（仅心跳 `Type=100 Cmd=100`）、
  `aos_basic_status`、`aos_motion_state`、`aos_gait_known`、`gait_speed_window`。
- 每一项失败都带 `fix` 字段（例如 `sudo systemctl enable --now multicast-relay.service`）。
- 写 `/tmp/m20_preflight.json`（含 `version / generated_at / host / checks`）。
- **live 启动闸门**：`m20.yaml` 默认 `require_preflight: true`，`m20_bridge` 在
  `dry_run:=false` 时若报告缺失、过期（默认 600 s）、版本不符或任一 required 检查失败，
  就**拒绝启动**（`RuntimeError`）。这把 `commissioned:=true` 从「口头声明」变成
  「可核查的前置条件」——但它仍然不是安全认证。
- 该工具只发心跳，不发速度、不改模式/步态/服务状态。

## 6. 其余一并收紧的项

| 项 | 改法 |
| --- | --- |
| 控制周期 | `control_hz` 参数化，默认 20 Hz（官方建议 ≥20 Hz，本体 500 ms 超时） |
| 状态字段 | 继续强制 `MotionState=17`、`Direction=0`、`HES/Charge/Sleep=0`、已知步态 |
| 前后雷达选型 | `m20_cloud_to_scan` 视场收窄到 ±100°，几何上等价「只用前雷达的前向区域」而**不改** `send_separately`，见 §8 |
| Web/Android/D1 动作 | M20 launch 继续强制关闭 |
| 测试 | Python 单测 50 项（含 preflight 闸门、扇区、自车遮罩、模式矩阵、snap 策略） |

## 7. 本机验证证据（WSL Ubuntu 22.04 + ROS 2 Humble）

构建与测试：

```
git clone /mnt/d/repos/deamon2-m20 ~/m20_ws/src/jie_deamon
colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=ON   # exit 0
python3 -m unittest discover -s test                                        # Ran 51 tests ... OK
colcon test --ctest-args -R test_m20_tracker                                 # 5 tests, 0 errors, 0 failures
python3 test/ros_m20_smoke.py                                                # PASS: default zero, forward
                                                                             # tracking, target loss,
                                                                             # obstacle latch, input loss
```

调试过程记录（值得保留）：最初 `ros_m20_smoke.py` 报 `command stale`。加入
`counters`（commands/scans/statuses/armed_commands）与 `since_command/since_scan/since_status`
遥测后定位到：桥在**尚未收到任何 cmd_vel** 时就被 arm，0.3 s 宽限期一过即自动解除。
修复方向不是放宽超时，而是让 `arm()` 在速度流未就绪时**拒绝使能**，并让测试按操作员
习惯重试 arm。这些计数器与新鲜度字段保留在 `/m20/bridge_status` 里，现场排障可直接看。

**可移动模拟人仿真**（`ros2 run jie_deamon m20_follow_sim`，隔离 domain 83 +
localhost-only，`dry_run:=true`，不碰 AOS、不开控制 socket）。模拟点云包含半径 5 m 的
房间、宽 0.30 m 的人形簇、可选障碍与两条真实自车回波；人按 0.4 m/s 沿脚本路径行走，
每个场景由「操作者」重新选定目标。

| # | 场景 | raw cmd_vel | guarded cmd_vel | 结论 |
| --- | --- | --- | --- | --- |
| 01 | 人在正前 2.0 m | (0.4, 0, 0) | (0.3, 0, 0) | 前进 |
| 02 | 人停在跟随距离 1.2 m | (0, 0, 0) | (0, 0, 0) | 保持 |
| 03 | 人退到 0.9 m | (-0.12, 0, 0) | (0, 0, 0) | 算法要后退，门控禁止后退 |
| 04 | 人移到左前 (1.0, 1.0) | (-0.078, 0, 0.774) | (0, 0, 0.6) | 左转，限幅生效 |
| 05 | 人移到右前 (1.0, -1.0) | (-0.082, 0, -0.783) | (0, 0, -0.6) | 右转，符号正确 |
| 06 | 小偏角 (1.4, 0.35) | (0.101, 0, 0.241) | (0, 0, 0) | 低于步态下限 → fail-closed 置零 |
| 07 | 目标消失 | (0, 0, 0) | (0, 0, 0) | 目标丢失即零速 |
| 08 | 加入真实自车回波 | (0.4, 0, 0) | (0.3, 0, 0) | **不再误停，可 arm** |
| 09 | 正前 0.5 m 障碍 | (0.4, 0, 0) | (0, 0, 0) | 解除使能，reason=`invalid scan or obstacle` |
| 10 | 后方 -0.6 m 障碍 | (0.4, 0, 0) | (0.3, 0, 0) | 扇区外，不误触发 |
| 11 | 常规模式 `ControlUsageMode=0` | (0.4, 0, 0) | (0.3, 0, 0) | **`cmd=21` 归一化路径可用** |
| 12 | 导航模式 `ControlUsageMode=1` | (0.4, 0, 0) | (0.3, 0, 0) | `cmd=25` SI 路径可用 |

`12/12 scenarios passed`。复现命令：

```bash
source /opt/ros/humble/setup.bash && source ~/m20_ws/install/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run jie_deamon m20_follow_sim --report /tmp/m20_follow_sim_report.json
```

**注意**：Humble 只验证「算法+门控逻辑」，不能替代 Foxy/ARM64 与实机验证；
本机 OpenCV 版本、RMW、CPU 架构都与 GOS 不同。

## 8. 前后双雷达：融合点云 vs 仅前雷达

完整分析见 [LIDAR_CHOICE_ANALYSIS.md](LIDAR_CHOICE_ANALYSIS.md)。结论：
**保留融合点云，但把有效视场收窄到前向扇区**，不改 `send_separately`
（官方明确改它会「导航、定位功能将无法使用，自主充电功能部分受限」）。

- `m20_cloud_to_scan.angle_min/max` 收到 ±100°。扇区内
  `pointcloud_to_laserscan` 每个 bin 只保留最近点，前雷达对同一物体距离恒小于后雷达，
  因此扇区内的胜出者必为前雷达 —— 几何上等价「只用前雷达的前向区域」。
- 守卫扇区与自车遮罩同步收窄/放大（§4）。
- 前雷达失效检测：扇区有效 bin 数不足即 fail-closed；进一步可用
  `Type=1002 Cmd=5` 的 `DevEnable.Lidar.Front != 1` 主动 disarm（待现场接入）。

## 9. 官方文档漂移（19 篇比对）

见 [DOC_DRIFT_2026-09-09.md](DOC_DRIFT_2026-09-09.md) 与
[doc_drift_summary.json](doc_drift_summary.json)。结论：**19 篇中仅 2 篇有真实漂移**，
且 11 项关键事实（Cmd=25/21 模式限制、步态速度表、500 ms 超时、1101/5 切模式、
`/LIDAR/POINTS` 需 su 且仅机器人主机传输、`send_separately`、relay 默认关闭、
BasicStatus 2 Hz+心跳、`/NAV_CMD` 冲突、机身尺寸、V1.1.7/V1.1.8 条目）**无一变化**。

- `development`：V1.2.1 → V1.3.0，新增 1.2.7 自定义灯语 `(1101,14)`、
  1.2.10 GPS 时间同步 `(1101,16)`、1.2.11 外部时间同步 `(1101,15)`；
  1.4.2 响应新增 `Confidence` / `Relocation`；`Sleep` 由 bool 改为 int。
- `updates`：新增两段 `V1.2.0.1` 条目（M20 18 条、M20 Pro 31 条）。

对本次改动的意义：`Sleep` 类型变化**不影响**本门控——它只要求 `Sleep == 0`
（int 比较），旧快照的 bool 写法本来就会被 `type(...) is not int` 拒绝，属 fail-closed 方向。
`Confidence` / `Relocation` 可用于后续定位健康诊断，当前未接入。

## 10. 仍未清除的项（必须实机）

1. **从未向 AOS 发送过速度**：Cmd=21/25 的实际响应、单位、符号、最低速度全部待实测。
2. 本体 500 ms 速度超时、实际停车距离、硬急停行为未实测。
3. `axis_max_x/y/yaw`（归一化满量程）需实测标定。
4. 自车遮罩余量、停止区尺寸需按实测腿部扫掠与实车回波复调。
5. PTP 同步（`rsdriver ptp=0x1`、时间差）未验证。
6. 控制权接管（停 planner、确认无其它速度源）仍需人工执行并被 preflight 检查。
7. `snap` 策略、`enable_lateral`、`allow_reverse` 三项默认关闭，需逐项实机接受。
