# M20 Pro 最小验收 checklist（GOS 可执行）

适用：`deamon2-m20`（硬阻塞清除后）。目标固件 ≥ V1.1.7，建议 V1.1.8（含「开机后没有点云」
与「外部跨版本 ROS2 监听导致内部 ROS2 崩溃」修复）。

**执行规则**

- 每一条都有「命令 / 判据 / 不通过怎么办」。**判据不满足就停下来**，不要跳过。
- 全程保留原厂遥控器 + 硬急停 + 第二名观察员。
- 任何一步出现非预期运动，立即硬急停，并在记录表登记。
- 本清单只覆盖最小集合；不通过项未清零前，**不得进入下一步**。

---

## 0. 前置条件（人工）

| # | 项目 | 命令 / 动作 | 判据 |
| --- | --- | --- | --- |
| 0.1 | 固件版本 | 在 GOS `cat /opt/robot/.../version*`；或用 APP 查看 | ≥ V1.1.7 |
| 0.2 | 场地 | 清空 3 m × 3 m，地面平整、无台阶/玻璃门/负障碍 | — |
| 0.3 | 机器人 | 用原厂控制端起立，站立稳定 | `MotionState=17` |
| 0.4 | 代码 | `cd ~/m20_ws/src/jie_deamon && git log --oneline -1` | 记录 commit |
| 0.5 | 构建 | `colcon build --packages-select jie_deamon` | exit 0 |

```bash
# 每次验收都从干净的 root 环境开始
sudo -i
export ROS_DISTRO=foxy
source /opt/robot/scripts/setup_ros2.sh
source /home/user/m20_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 1. 只读预检（必须全绿）

```bash
ros2 run jie_deamon m20_preflight.py --seconds 12 --require-scan
echo "exit=$?"   # 必须是 0
cat /tmp/m20_preflight.json | python3 -m json.tool | head -60
```

| # | 检查项 | 判据 | 不通过怎么办 |
| --- | --- | --- | --- |
| 1.1 | `root` | euid=0 | `/LIDAR/POINTS` 官方要求 su，必须 root 运行 |
| 1.2 | `multicast_relay` | active + **enabled** | `sudo systemctl enable --now multicast-relay.service` |
| 1.3 | `lidar_cloud` | ≥3 帧、~10 Hz、字段含 x/y/z、age<0.5 s | 查 `rsdriver.service`、relay、PTP |
| 1.4 | `scan_topic` | `/m20/scan` 有数据、frame=`lidar_link` | 先起 `m20.launch.py`（dry-run） |
| 1.5 | `planner_idle` | `planner.service` 非 active | `sudo systemctl stop planner.service` |
| 1.6 | `no_nav_cmd_publisher` | `/NAV_CMD` 发布者 = 0 | 找出并停掉其它速度源 |
| 1.7 | `aos_link` | 6 s 内 ≥2 条 BasicStatus | 查 AOS `10.21.31.103:30000` 路由 |
| 1.8 | `aos_motion_state` | `MotionState=17` | 原厂控制端起立 |
| 1.9 | `aos_gait_known` | Gait ∈ {4097,4099,12290,12291} | 原厂控制端选步态 |
| 1.10 | `time_sync` | 报告值（info，非阻塞） | 记录 `rsdriver` 日志 `ptp=0x1` 与时间差 |

> `1.10` 是 info 级：不阻塞启动，但**必须人工记录**。扫描时间戳依赖 PTP。

---

## 2. 传感器与切片验收（dry-run）

```bash
# 终端 A
ros2 launch jie_deamon m20.launch.py dry_run:=true commissioned:=false
# 终端 B（同环境）
ros2 run jie_deamon m20_cloud_probe.py --seconds 12
ros2 topic echo --once /m20/scan_report
```

| # | 判据 | 不通过怎么办 |
| --- | --- | --- |
| 2.1 | 点云 ~10 Hz、9 万~10 万点/帧 | 按 §1.3 |
| 2.2 | `/m20/scan` ~10 Hz、`frame_id=lidar_link`、约 401 bins | 检查 `m20_cloud_to_scan` 参数是否生效 |
| 2.3 | `scan_report.sector_bins ≥ 20` 常态成立 | 前雷达失效或遮挡；**不要**调小 `min_sector_bins` 来掩盖 |
| 2.4 | 清空环境时 `stop_hits=0`、`valid=true` | 记录 `closest_stop`/`closest_self` 坐标，判断是自车回波还是真障碍 |
| 2.5 | 人站在正前 1.5 m，`stop_hits` 不误报；人走进 0.6 m，`stop_hits>0` | 调 `stop_front/stop_half_width`，**不要**关掉停止区 |
| 2.6 | RViz（可选）能看到点云与 `/m20/scan` | X11 转发，软件 OpenGL，仅供观察 |

**记录**：`/m20/scan_report` 里的 `closest_self` 坐标 —— 若出现自车回波，用它校准
`self_margin_back/left/right`（默认 back 0.14、left/right 0.07）。

---

## 3. 使用模式与步态（**任务四结论落地**）

```bash
ros2 run jie_deamon m20_status_probe.py --seconds 6
```

| 场景 | 期望 |
| --- | --- |
| `ControlUsageMode=0`（常规） | 桥自动选 `Cmd=21`（归一化），**不需要改手柄模式** |
| `ControlUsageMode=1`（导航） | 桥自动选 `Cmd=25`（m/s），**需要手柄/APP 切到导航模式** |
| `ControlUsageMode=2`（辅助） | 默认拒绝；如需使用加 `allow_auxiliary_mode:=true`（不建议，辅助模式带避障介入） |

**结论：是否需要把手柄从「基础」改成「导航」？**

- 手柄上的「基础/楼梯/平地」是**步态**，不是使用模式；使用模式是 常规/导航/辅助。
- **如果走常规模式（出厂默认，`ControlUsageMode=0`）：不需要切模式。** 桥用 `Cmd=21`
  归一化指令，这正是手柄平时用的通道 —— 代价是**操作员在跟随期间必须松手不要碰摇杆**
  （同一个轴指令通道，手动输入会与算法竞争）。
- **如果走导航模式（`ControlUsageMode=1`）：必须切。** 切法：APP 的 R1/R2 自定义键
  （V1.1.8+）或 `Type=1101 Cmd=5 Mode=1`。代价：切模式会**把步态重置为 0x1001**，
  且导航模式会启用原厂导航栈（必须先停 `planner.service`），风险更高。
- **验收建议**：先在常规模式用 `Cmd=21` 完成 §5~§7；确认通过后再评估是否需要导航模式。

| # | 判据 | 不通过怎么办 |
| --- | --- | --- |
| 3.1 | `BasicStatus` 完整且 2 Hz | 固件过旧 |
| 3.2 | 桥日志出现 `control command changed: None -> 21`（常规）或 `-> 25`（导航） | 模式与 `control_profile` 冲突，看 `bridge_status.reason` |
| 3.3 | `bridge_status.axis_notes` 为空 | 有 `y axis unusable` 属正常（Y 默认关闭）；出现 `x/yaw unusable` 说明限幅低于步态下限，需调 `max_vx/max_wz` |
| 3.4 | 记录当前步态与 `GAIT_TABLE` 是否一致 | 固件差异 → 按实测更新 `m20_adapter/core.py::GAIT_TABLE` 并跑单测 |

---

## 4. 控制权接管（人工确认）

| # | 项目 | 判据 |
| --- | --- | --- |
| 4.1 | `planner.service` 已停 | `systemctl is-active planner.service` → `inactive` |
| 4.2 | 无自主充电任务 | APP/状态确认充电空闲（`Charge=0`） |
| 4.3 | 无其它速度源 | `/NAV_CMD` 发布者 0；无其它 basic_server 客户端 |
| 4.4 | 保留服务 | `basic_server`、`rl_deploy`、雷达驱动、时间同步**不要停** |
| 4.5 | 恢复方案已记录 | 写下如何把 planner/模式/步态恢复原状 |

---

## 5. 首次 live：只发心跳与零速（**不 arm**）

```bash
ros2 launch jie_deamon m20.launch.py dry_run:=false commissioned:=true
```

| # | 判据 | 不通过怎么办 |
| --- | --- | --- |
| 5.1 | 启动**不报** `preflight gate rejected` | 重跑 §1，报告 10 分钟内有效 |
| 5.2 | `bridge_status.reason` = `not armed`，`velocity=[0,0,0]` | 检查是否被其它源 arm |
| 5.3 | AOS 无 `ErrorCode` 回包 | 查协议字段（`bridge_status` 会写 `basic_server rejected`） |
| 5.4 | 机器人**完全静止** 30 s | 硬急停，检查是否误发速度 |
| 5.5 | `counters.commands` 持续增长（≥8 Hz） | 算法/scan 链路问题 |

> 这一步只证明「通道能收发零速」，**不是运动验收**。

---

## 6. 单轴最小速度验证（逐轴、逐符号）

前置：清空场地，人手不离硬急停，速度上限先降到 `max_vx:=0.1 max_wz:=0.2`。

```bash
# 目标：先只验证 X 轴
ros2 topic pub --once /robot_nexus/target geometry_msgs/msg/Point '{x: 2.0, y: 0.0, z: 0.0}'
ros2 service call /robot_nexus/set_moving std_srvs/srv/SetBool '{data: true}'
ros2 service call /m20/arm std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /m20/bridge_status      # 观察 reason=armed, velocity 非零
# 观察机器人：是否向前、速度是否与命令量级一致
ros2 service call /m20/arm std_srvs/srv/SetBool '{data: false}'
```

| # | 轴 | 判据 | 记录 |
| --- | --- | --- | --- |
| 6.1 | X 正 | 向前移动；命令 0.1 m/s 与实测位移一致 | 单位是否真是 m/s |
| 6.2 | X 负 | 默认 `allow_reverse:=false` 时**不动**；开启后向后 | 是否需要倒车 |
| 6.3 | Yaw 正 | 逆时针（左转） | 符号 |
| 6.4 | Yaw 负 | 顺时针（右转） | 符号 |
| 6.5 | 最低有效速度 | 按步态表逐档测 0.15/0.2/0.35/0.5… 找实际能动的阈值 | 与官方表是否一致 |
| 6.6 | Y 轴（可选） | 需先 `axis_enable_y:=true` 且 `enable_lateral:=true` | 符号与幅值 |
| 6.7 | `axis_max_*` 标定 | 常规模式下 `Cmd=21` 归一化满量程实测 | 用实测值替换 2.0/1.0/2.0 |

**若 6.5 发现「小误差不动、一动就跳」，** 说明 `gait_min_policy=zero` 的分段特性：
先用 `gait_min_policy:=snap` 验证平滑性，再决定是否接受。

---

## 7. 跟随功能验收（dry-run → live）

```bash
# 7.1 dry-run（不开发送）
ros2 launch jie_deamon m20.launch.py dry_run:=true
# 7.2 人在 2 m 正前方，选定目标并观察 raw/guarded
ros2 topic pub --once /robot_nexus/target geometry_msgs/msg/Point '{x: 2.0, y: 0.0, z: 0.0}'
ros2 service call /robot_nexus/set_moving std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /m20/cmd_vel_raw        # 应为 +x
ros2 topic echo /m20/cmd_vel_guarded    # dry-run 恒为 0
```

| # | 场景 | 期望（dry-run 先看 raw，再 live 看实车） |
| --- | --- | --- |
| 7.1 | 人在正前 2.0 m | raw `+x`；live 时向前 |
| 7.2 | 人停在 1.2 m | raw `x≈0` |
| 7.3 | 人向左移 0.5 m | raw `wz>0`（左转） |
| 7.4 | 人向右移 0.5 m | raw `wz<0`（右转） |
| 7.5 | 人消失 | raw=guarded=0（不追残留目标） |
| 7.6 | 人快速穿过 | 跟踪是否丢失；丢失后应零速 |
| 7.7 | 正前 0.6 m 出现障碍 | 门控 disarm，`reason=invalid scan or obstacle`，guarded=0 |
| 7.8 | 障碍移除后 | **不自动恢复**，需人工 `/m20/arm` |
| 7.9 | 走廊居中（若启用 lateral） | 双侧墙都可见才输出，幅值 ≤ `lateral_max` |

**离线预演**：以上 12 个场景已由 `m20_follow_sim` 在 WSL/Humble 上跑通（12/12），
可在 GOS 上用隔离域复核：

```bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 run jie_deamon m20_follow_sim --report /tmp/follow_sim.json
```

---

## 8. 故障演练矩阵（每一项都要实测停车）

| # | 注入 | 期望行为 | 实测停车距离 |
| --- | --- | --- | --- |
| 8.1 | 停算法（`set_moving false`） | 0.3 s 内归零 | ___ m |
| 8.2 | 停桥进程（`Ctrl+C`） | 进程退出前发零速；本体 500 ms 超时兜底 | ___ m |
| 8.3 | 杀桥进程（`kill -9`） | 只能靠本体 500 ms 超时 | ___ m |
| 8.4 | 断点云（停 relay） | 0.5 s 内 disarm | ___ m |
| 8.5 | 断网（拔网线/关 AOS 路由） | UDP 错误 → disarm | ___ m |
| 8.6 | 硬急停 | 关节断电 | — |
| 8.7 | 软急停（原厂） | 关节断电，桥应 disarm（`HES`） | — |
| 8.8 | 切回手动（手柄介入） | 算法命令被手动覆盖，记录优先级 | — |
| 8.9 | 触发充电（`Charge≠0`） | disarm | — |
| 8.10 | 遮挡前雷达 | `sector_bins` 骤降 → `valid=false` → disarm | — |
| 8.11 | 遮挡后雷达 | 前向扇区不受影响（记录是否仍有回波变化） | — |
| 8.12 | 时间跳变/PTP 失锁 | 扫描 age 异常 → disarm，**不要**改时间戳掩盖 | — |

---

## 9. 性能与资源

| # | 命令 | 判据 |
| --- | --- | --- |
| 9.1 | `taskset -c 4-7 ros2 launch jie_deamon m20.launch.py` | 绑大核（官方建议） |
| 9.2 | `pidstat -p $(pgrep -f robot_nexus) 1 30` | CPU/内存无持续爬升 |
| 9.3 | `ros2 topic hz /m20/scan /m20/cmd_vel_raw` | 均 ~10 Hz；桥 20 Hz |
| 9.4 | 端到端延迟 | 扫描时间戳 → 发包时间，记录均值/最大值 |
| 9.5 | `vcgencmd measure_temp`（或 sensors） | 温度在安全范围 |

---

## 10. 收尾与恢复

```bash
ros2 service call /m20/arm std_srvs/srv/SetBool '{data: false}'
ros2 service call /robot_nexus/set_moving std_srvs/srv/SetBool '{data: false}'
# 确认机器人实测静止后再退出 launch
ps -eo pid,user,args | grep -E '[m]20_bridge|[r]obot_nexus|[p]ointcloud_to_laserscan'
```

| # | 项目 | 判据 |
| --- | --- | --- |
| 10.1 | 机器人静止 | 目视确认 |
| 10.2 | 桥/算法进程退出 | 无残留 |
| 10.3 | 原厂 planner/模式/步态恢复 | 与 §4.5 记录一致 |
| 10.4 | 记录归档 | 下表填完 |

---

## 验收记录表

| 项目 | 记录 |
| --- | --- |
| 日期 / 操作者 / commit | |
| 固件 / 系统 / ROS / RMW 版本 | |
| AOS/NOS/GOS 实际 IP；relay 所在主机与接口 | |
| 点云话题/发布者/QoS/frame/频率 | |
| `/m20/scan` bins/frame/频率 | |
| BasicStatus 原始回包 | |
| 使用模式（0/1/2）与所选 `command_kind`（21/25） | |
| 当前步态 / 最小有效速度 / 各轴符号 | |
| `axis_max_*` 实测标定值 | |
| 自车回波坐标 / 调整后的 `self_margin_*` | |
| 停止区尺寸 / 实测停车距离（§8 逐项） | |
| 控制权接管与恢复记录 | |
| 故障演练结果（§8 逐项） | |
| GOS CPU/内存/温度/端到端延迟 | |
| 结论 | □ 通过 □ 有条件通过 □ 不通过 |

---

## 一页速查：不通过就不能往下走的红线

1. `m20_preflight.py` 任一 required 检查失败。
2. `scan_report.sector_bins < min_sector_bins` 或 `stop_hits` 在清空场地时非零。
3. `bridge_status.reason` 非 `armed` 时出现非零 `velocity`。
4. 单轴符号与文档不符。
5. 任一故障注入未能在预期时间内归零/停车。
6. 无法确认原厂 planner/充电已空闲。
