# NEW_CHASE — M20 适配（NEW-06 首次功能实现：仅算法/NavCmd预览）

> ## ⚠️ 置顶警告（2026-09-17 验收：PARTIAL / NOT ACCEPTED）
> - **默认 launch 的 HTTP 首页当前返回 404**：`launch/new_chase_preview.launch.py` 第51行计算了
>   `web_root = os.path.join(get_package_share_directory('jie_deamon'), 'web')`，但 robot_nexus
>   参数字典（78-105行）**没有传入 `'web_root': web_root`**；installed share 下 web/index.html
>   实际存在，缺的只是参数传递。（此前文档/注释声称"web_root 已传入"不实，系本适配自身笔误，
>   非上游问题；NEW-10 测试是直接以 binary 显式 web_root 启动，因此未覆盖默认 launch 此缺陷。）
> - 修复额度（NEW-06 共3轮）已耗尽，本轮仅文档收尾，**禁止再改任何代码/launch/测试**。
> - 完整验收状态与决策项见 `/home/user/new_chase/evidence/ACCEPTANCE-20260917.md`。
> - **不要据此认为默认界面"下一条命令即可正常使用"；本阶段物理控制未实现、无 live 入口，
>   不存在任何开启实机的指令。**

基线: 上游 https://github.com/6-robot/jie_deamon @ 9403185c0cf99a0b415ddd4f3b4215b44dd0809c
本包路径: /home/user/new_chase/src/jie_deamon
本阶段结论: **只预览，绝不实机控制。物理运动功能未实现，机器不会运动。**

## 1. 当前状态（重要，不要误判）

- [x] 算法预览：跟随/选点/势场避障算法在 LaserScan 上运行，速度结果发布到**预览话题**（无真实底盘消费者）
- [x] 标准 NavCmd 消息转换：`to_nav_values` 纯函数 + `nav_cmd_preview` 节点（drdds/msg/NavCmd）
- [ ] **Web 默认入口 404（见置顶警告）**：仅当显式传 web_root（如 NEW-10 测试以 binary 直启）
      时界面可用；默认 launch 未传参数
- [ ] Web 界面功能（横幅/选点/selected标记等）在显式 web_root 下也未做完整人工验收
- [x] WS 握手 Sec-WebSocket-Key 严格校验（消除 popen 命令注入面）；JSON 数值解析异常捕获
- [x] **SDK export→ROS 链路已验证**（NEW-09 r1 + NEW-11 实跑，日志 evidence/ 下；
      证据是当时窗口，非长期稳定性保证）
- [x] NEW-11: launch 默认 cloud_topic 已切换为 /new_chase/points（override 保留）
- [ ] **原生 /LIDAR/POINTS 直连 ROS 仍未打通**（当前靠 export 中转；合成 LaserScan
      路径同样未承诺可用，需 tester 实测）
- [ ] **尚未实机运动**：本阶段 robot_nexus 不创建 /cmd_vel、/d1_cmd 发布者；
      nav_cmd_preview 不发布 /NAV_CMD，仅 /new_chase/nav_cmd_preview 预览（SI 单位 +
      basic gait 4097 死区限幅）；**尚无运动跟随，preview 不等于完成**
- [ ] M20 官方运动模式导航模式/独占控制权：**未验证**（TODO 后续任务）
- [ ] 几何参数（body frame 0.46/0.46/0.31/0.31、rect 0.7、follow 1.2）为**静态估计未标定**，不作为实机安全证明
- [ ] 合并双雷达机架/场景/失效标定未完成，双雷达独立健康未证实
- [ ] 单雷达失效场景未验证
- [ ] root 运行 SDK 需 SHM 权限且 export 无远端 IP 白名单（本机网段内多机可达，部署需自行限制）
- [ ] 人工安全检查流程、实机模式控权验证：后续完成

## 2. 启动预览步骤（两终端；本任务未运行过任何节点/服务/运动）

**终端1（root，独立小进程，仅接点云；不加载ROS环境，不daemon/systemd，不整Web提root）：**

```bash
sudo env -i HOME=/root USER=root \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /home/user/new_chase/bin/drdds_cloud_export --continuous
# 验收用: 追加 --seconds 25
# 密码由用户在终端自行输入，不得把明文密码写进任何文件
```

**终端2（普通 user，本包预览，默认 cloud_topic=/new_chase/points）：**

```bash
/home/user/new_chase/src/jie_deamon/scripts/run_preview.sh
# 常用参数:
#   use_cloud_converter:=false          # 合成LaserScan测试（跳过pointcloud_to_laserscan）
#   cloud_topic:=/LIDAR/POINTS          # 原生主题（尚未打通，仅测试用途）
#   enable_web:=false                   # 关Web
# ROS_DOMAIN_ID 默认0；测试域: NEW_CHASE_ROS_DOMAIN=42 run_preview.sh
```

- **停止方式**：终端各自 `Ctrl-C` 正常析构退出；**不 pkill、不清理 SHM**。
- **仅预览/NavCmd 专用 topic**（/new_chase/*），无真实 /NAV_CMD 发布；UI 上显示的速度是算法预览值，**不是机器人实际速度**。
- Web 界面地址 `http://10.21.31.104:8080`（端口8080）——**默认 launch 下首页 404（见置顶警告）**；
  仅显式传 web_root 时界面可用；界面功能无论哪种方式均未做完整人工验收。
- 以上20项纯函数/单点测试已过，**不构成完整验收**；当前仍在等待 ROS 集成测试；运动功能尚无 live 入口可写。

脚本约束：`env -i` 清洁环境，只 source /opt/ros/foxy/setup.bash + 本 install，**不用 bash -lc**（避免加载旧workspace），保留 HOME/USER/TERM/LANG/PATH 最小集。

## 3. 话题契约（launch 默认，均可参数覆盖）

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| /new_chase/points | PointCloud2 | 订阅 | **NEW-11默认**点云输入（SDK drdds_cloud_export 输出，NEW-09 已验证；override 保留可指向 /LIDAR/POINTS 等测试主题——原生/LIDAR/POINTS 直连 ROS 尚未打通） |
| /new_chase/scan | sensor_msgs/LaserScan | 内部 | pointcloud_to_laserscan 输出 → robot_nexus（SensorDataQoS） |
| /new_chase/cmd_vel_raw | geometry_msgs/Twist | robot_nexus→ | **预览raw**：算法限幅输出（vx≤0.3 前进向, vy=0, wz≤0.6）。非 /cmd_vel |
| /new_chase/nav_cmd_preview | drdds/NavCmd | nav_cmd_preview→ | **标准NavCmd预览**：SI单位 x_vel/y_vel/yaw_vel；header.stamp=ROS now；header.frame_id=递增uint64序列；gait4097阈值下置零（x<0.2/y<0.35/yaw<0.5→0）；20Hz；raw超时0.3s→全零 |

### NavCmd 契约（/opt/ros/foxy/share/drdds/msg/NavCmd.msg）
```
NavCmd:  header(MetaType)  data(NavCmdValue)
MetaType: frame_id(uint64, 递增序列)  stamp(builtin_interfaces/Time, ROS now)
NavCmdValue: x_vel/y_vel/yaw_vel (float32, SI m/s & rad/s, 非归一化)
```
固定 basic 预览：仅 basic gait 4097 有效阈值语义；**未验证导航模式/独占控制**，不订阅 MOTION_INFO。

### robot_nexus 新参数（m20_preview=true 时生效）

| 参数 | 默认 | M20取值(launch) | 说明 |
|------|------|------|------|
| m20_preview | false | true | 预览总开关；false=上游D1行为不变 |
| scan_topic | /scan | /new_chase/scan | |
| expected_scan_frame | "" | launch arg scan_frame_id | 非空则frame不符拒绝 |
| rotation_angle | pi | 0.0 | 上游硬编码180°参数化 |
| follow_distance | 0.4 | 1.2 | |
| rectangle_width | 0.35 | 0.7 | |
| robot_frame_front/back/left/right | 0.15/0.35/0.15/0.15 | 0.46/0.46/0.31/0.31 | 静态估计未标定 |
| scan_stale_max / scan_future_max | 0.5 / 0.1 | 同 | 时间戳校验阈值(s) |

## 4. 预览安全语义（已实现）

- 未人工选点(target_selected=false)不跟随默认目标；Web双击/HTTP/WS set_target 才置 selected（非法输入HTTP 400 / WS忽略）
- 目标当前帧无匹配 → 零速 + selected/moving 复位，**需重新选点+重新使能**，目标回来也不自动续追；速度用当前新质心
- scan 空/无合法range → 零速+失能；scan 过期>0.5s/未来>0.1s、frame不符 → 拒绝该帧
- 看门狗：单调时钟 0.5s 无有效 scan → 立即零速发布 + moving=false + selected=false
- active/moving 关闭后下一帧内零速（并广播点云供选点）
- 预览限幅 vx∈[0,0.3]（保守禁后退）、vy=0（0.3<0.35阈值恒无效）、wz∈[-0.6,0.6]；阈值下由 to_nav_values 置零，**不向上放大**；上游 min_speed=0.06 提升逻辑在 m20 下禁用
- WS handshake key 校验失败直接断开；数字解析捕获异常，坏输入不崩进程

## 5. 给 tester 的测试接口

### 纯函数（无ROS导入即可 unittest）
```python
import sys; sys.path.insert(0, '/home/user/new_chase/src/jie_deamon/scripts')
from nav_cmd_preview import to_nav_values, is_fresh, PREVIEW_MAX_*, GAIT_MIN_*, CMD_FRESH_WINDOW
# 用例示例:
# to_nav_values(0.25,0,0.55) == (0.25,0,0.55)   # 有效区间内直通
# to_nav_values(0.1,0,0.3)   == (0,0,0)        # 低于gait阈值置零
# to_nav_values(-0.3,0,0)    == (0,0,0)        # 禁止后退
# to_nav_values(1.0,1.0,2.0) == (0.3,0,0.6)    # 限幅
# to_nav_values(float('nan'),0,0) == (0,0,0)   # 非有限全零
# is_fresh(10.0, 9.8) True; is_fresh(10.0, 9.6) False
```
python: `/usr/bin/python3`（GOS 系统解释器，drdds 导入在 main() 内，unittest 不触发）。
注意：nav_cmd_preview.py 启动日志中"普通ROS链路未通"文案为旧期表述（该文件不在
NEW-11 授权范围内，未改）；链路现状以本文档为准——export→ROS 已验证，
原生 /LIDAR/POINTS 直连 ROS 未通。

### tracker 门控接口（SharedState, C++）
- `selectTarget(x,y)`：人工选点（置selected）；`updateTrackedTarget(x,y)`：算法质心（不置selected）；`clearTracking()`：丢目标复位
- `m20_preview` 原子开关；Web scan_data 新字段：`m20_preview/is_selected/is_tracking_valid`

### launch 验证命令
```bash
ros2 launch jie_deamon new_chase_preview.launch.py use_cloud_converter:=false
# 然后由tester自合成 LaserScan 发布到 /new_chase/scan（frame_id与scan_frame_id一致或留空）
```

## 6. 禁止事项（本阶段）

- 不创建/使用 /NAV_CMD /MOTION_STATE /GAIT /cmd_vel /d1_cmd 实际发布者
- 转换节点无 live/arm/enable_motion 参数（代码硬性拒绝指向真实话题；预览话题被重映射到真实控制话题时 C++/Python 均 exit 2 拒绝启动）
- **nav_cmd_preview 本节点禁止所有话题重映射**（第2轮起）：argv 中除 `__node/__name/__ns` 精确元规则外的任何规则（节点限定/通配/rostopic://）一律 exit 2；预览话题改名仅可经 `preview_topic` 参数（严格 /new_chase/* 白名单），raw_topic 参数校验不得指向真实控制话题；不再声称支持完整 remap 解析
- 几何参数仅为预览初值；**未接收真实ROS点云、未实机验证**，安全证明与官方模式控权需后续任务完成
- 机体排除区（0.46/0.46/0.31/0.31）仅用于算法内部障碍排除的静态估计，**不代表实机碰撞/安全验证通过**

## 7. 修复记录

### 第1轮（NEW-06 修复第1轮，2026-09-17）

- P0: robot_nexus 补 create_subscription（**修正表述：这是本适配第1轮实现的遗漏，并非上游仓库问题**）；raw 发布话题创建前 rclcpp::expand_topic_or_service_name + rcl_remap_topic_name 解析（Foxy 无 resolve_topic_name，已核对）+ 创建后 rcl_publisher_get_topic_name 复核，非法配置 exit 2；launch raw_topic 双向接入
- P0: nav_cmd_preview 创建前 expand+手动应用 -r/--remap 规则 + 创建后 Publisher.topic_name 复核，非法 exit 2
- P1: launch 计算了 web_root=share/jie_deamon/web（**第2轮记录当时表述为"已传入参数"，
  实际未传——2026-09-17 验收查明默认首页 404 根因，见置顶警告；表述更正，不归咎上游**）；
  expected_scan_frame（默认 lidar_link）与 converter target_frame(scan_frame_id，默认空) 分离
- P1: scanCallback 拒收分支统一 stopPreview（当帧零速+失能+清跟踪+广播同步 Web）；时间戳异常 catch 不 crash
- P1: tracker 元信息/范围外 range/非法点过滤，全非法帧清缓存防 UI 残影且不计有效 scan；速度非 finite 全零+清跟踪
- P1: 看门狗增加 !active/!moving 持续零发布（<=100ms），无扫描启动不清新选择；selectTarget 关 moving 并递增 generation 防旧 scan 覆盖新选点；未选点拒绝 set_moving=true（HTTP 400 / WS 忽略）
- P1: extractNumberStrict 完整数字 token 校验+分隔符检查（窄字段校验，非完整 JSON 解析器）
- P1: run_preview.sh source 期间 set +u；显式 RMW_IMPLEMENTATION=rmw_fastrtps_cpp、ROS_LOCALHOST_ONLY=0；env -i 杜绝 _old 残留
- P2: is_fresh 校验 finite/window>=0/0<=delta<=window；异常路径保证 rclpy.shutdown；to_nav_values 处理 OverflowError
- 质量授权范围内 LF/去尾空白统一

### 第2轮（NEW-06 修复第2轮，2026-09-17）

- P0(1): rcl_remap_topic_name 第二参数补 context 全局重映射规则（`rcl_context_t.global_arguments`，rcl/context.h:112；经 NodeBaseInterface::get_context()→Context::get_rcl_context()，rclcpp/context.hpp:221-222、node_base_interface.hpp:70-71；按 node_opts.use_global_arguments 决定是否传入），CLI -r 在创建前即被拦截；创建后断言保留为兜底
- P0(2): nav_cmd_preview 完全禁止话题重映射：main() 入口先扫原始 argv（fail-closed，仅放行 __node/__name/__ns 精确元规则，launch 生成的 -r __node:=xxx 属此），其余规则（节点限定/通配/rostopic:///悬空选项）一律 exit 2 不创建发布者；预览话题仅经 preview_topic 参数白名单改名；不再近似支持 FQN remap
- P1(3): extractNumberStrict 允许 token 前后标准 JSON 空白（{"x": 2 , "y": 0 } 合法）；isCompleteNumericToken 仅 '-' 符号、禁止前导零（01/-01）、"+1" 拒绝；1e/1..2/1junk/NaN 仍拒绝
- 其他: robot_nexus.cpp 注释修正（"上游遗漏scan订阅"表述不实，系第1轮适配实现遗漏，非原仓库问题）；scripts/__pycache__（上轮派生物）保留未删且不再产生（本轮测试一律 python -B，且不写 GOS /tmp 测试文件）

### NEW-11（小型集成，2026-09-17，不改功能逻辑）

- launch: `cloud_topic` 默认 `/LIDAR/POINTS` → `/new_chase/points`（override 保留）；converter 仍现成 pointcloud_to_laserscan、target_frame 空、expected_scan_frame 仍 lidar_link；无其他更改
- run_preview.sh: 仅用法注释更新（两终端流程/新默认主题/停止方式）
- docs: 启动步骤改为两终端（root export + user preview）；状态与注释更新为
  "export→ROS 已验证（NEW-09 r1），原生 /LIDAR/POINTS 直连 ROS 未通"；补充
  Web 8080 界面未验、UI 速度非机器人速度、20项测试不等于完整验收、运动无 live 入口；
  简洁说明：几何未标定/单雷达失效未验/root SDK 需 SHM 权限且 export 无远端 IP 白名单
- 证据: /home/user/new_chase/evidence/NEW-11-integration.md（实际改动+NEW-09 日志引用+后续验收清单）

### NEW-06 第3轮（最终轮，2026-09-17）

- P0: 参数读取修为 Foxy rclpy 真实 API（rclpy/parameter.py:144-151 `type_`/`value`/`get_parameter_value()`，**Parameter 无 as_string()，那是 C++ API**），`raw_topic`/`preview_topic` 均 `.value` 读取并严格校验类型必须为字符串（`_require_string_param`，非字符串/空值 exit 2）；全脚本审计无其他 `as_*` C++ API 误用
- graceful 生命周期: `KeyboardInterrupt` 视为 SIGINT 正常结束（exit 0）；cleanup 期间先 `SIG_IGN` 屏蔽 SIGINT 再析构（实测发现 launch 信号升级会在 destroy_node 中二次触发 KeyboardInterrupt 导致 process has died），结束后恢复原 handler；`rclpy.shutdown` 前置 `rclpy.ok()` 守卫防二次 shutdown；executor 显式 remove/shutdown；cleanup 异常逐项记录并 exit 2，不伪成功
- raw_topic 校验缺陷修复: 原 `try/except pass` 会吞掉"指向真实控制话题"的拒绝（伪通过），已分离 expand 失败与校验拒绝
- 日志文案更新: 启动日志改为"SDK export→ROS 链路已实证(NEW-09 r1, /new_chase/points)；本节点仅预览非运动控制"；20Hz/TTL0.3 原契约保持
- remap guard 不变: argv fail-closed 扫描 + 创建前白名单 + 创建后兜底断言全部保留
- 有界自测（1次，domain47/web关/converter关/10秒 timeout INT/UDP-only 配置）:
  - **已验证**: 参数 API 修复生效（无 AttributeError）、robot_nexus 与 nav_cmd_preview 正常启动、预览话题解析名 /new_chase/cmd_vel_raw→/new_chase/nav_cmd_preview、timeout 124 正常有界结束、无残留进程
  - **暴露并已修复但未经第二次launch复验**（单次运行预算已用完）: cleanup 被第二次 KeyboardInterrupt 打断（process has died exit -2），已加 SIGINT 屏蔽修复，待主代理亲跑确认
  - 日志: /home/user/new_chase/evidence/NEW-06-r3-launch.log
- 纯函数回归（python -B，stdin 执行未写远端文件）: ROUND3-PY-TEST PASS（21项，20Hz/TTL0.3 契约断言在内）
