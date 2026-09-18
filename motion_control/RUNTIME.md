# RUNTIME — motion_control 运行适配层（telemetry / inputs / runtime）

## 2026-09-18 主代理接管更新（优先于下文历史阶段说明）

### 首次实机验证的短窗口（第二次接管）

新增 `--motion-seconds`（默认10，有限数字且0<值≤10）。它仅缩短首次成功arm
后的会话期限，不改变进程 `--seconds`、人工arm、故障锁存或重新arm禁止。
Controller 对应参数 `session_seconds` 独立校验。首次输出试验可选择0.5秒窗口，
留出调度与传输余量；到期后的第一次tick只尝试零速，不再产生非零指令。
这不是硬实时或物理停车保证：长回调、进程卡死及丢包仍需机端看门狗/现场急停。

沿用 FOLLOW-RUNTIME / FOLLOW-RUNTIME-TEST 原任务，不增加子代理修复预算。
接管修复：

- `check_scan` 捕获畸形 ranges 索引/类型异常，返回 malformed，而不是抛出 KeyError。
- 启动发现等待优先响应停止，负数 raw 发布者计数立即拒绝，不等待 3 秒。
- Web 工作线程以有界的两个槽保留“尚未消费的失败 + 最新结果”；主循环先
  消费失败，再消费最新结果。已观察到的跟踪失效不能被随后好结果覆盖；重复
  seq 不刷新 TTL。这不是目标身份认证，也不保证检测到两次 HTTP 采样之间的失效。
- 退出先 disarm/尝试一次零速，再等待 Web 线程；线程退出异常记入 cleanup_errors，
  不跳过 sender/telemetry/executor/node 的回收。零速发送失败仍不能保证物理停车。

验证入口（clean env，仅 source `/opt/ros/foxy/setup.bash`，PYTHONPATH 包含项目）：

```
cd /home/user/new_chase/test
python3 -B -m unittest test_motion_guard test_nav_sender test_follow_runtime -v
# 另行设置批准 UDP-only XML、RMW 与 ROS_DOMAIN_ID=47：
python3 -B -m unittest test_follow_runtime_ros -v
```

隔离集成测试在初始化 DDS 前硬性校验域 47 和批准的 UDP-only 配置，不访问真实
Web/遥测、不更改现场服务。测试包含生产 Controller/Guard/Sender + 真实 ROS
LaserScan/Twist/NavCmd 发布订阅，以及生产 main 的 dry-run 子进程；后者只替换
HTTP fetch 并使用 `--no-telemetry`。没有通过放宽生产 live 域 0 限制来测试。
软件用例、实际命令和结果以 `evidence/FOLLOW-RUNTIME-TEST.md` 接管记录为准。

**验收边界**：域 47 的 `/NAV_CMD` 投递不等于机器狗响应，模拟状态不等于现场
控制权。未执行域 0 live、未 arm、未验证物理停止。几何/机体遮罩、停止距离、
前后雷达健康与现场急停仍须实机验收；不得据此自动设置 commissioned 标志。
下文“本任务未运行”等文字保留为原开发阶段历史，不表示当前测试覆盖状态。

本轮任务（NEW-RUNTIME，含修复1 retry）新增 4 个文件：

| 文件 | 职责 |
|---|---|
| `motion_control/telemetry.py` | 只读遥测 UDP 客户端（收 BasicStatus，唯一出站=1Hz 心跳） |
| `motion_control/inputs.py` | scan / web status 纯函数校验（metadata 问题判 valid=false，绝不抛穿） |
| `motion_control/runtime.py` | 有界 dryrun/live 运行入口（rclpy.init + 20Hz deadline 调度 + web worker） |
| `motion_control/RUNTIME.md` | 本文件 |

**其他全部只读**：core.py / nav_sender.py / 旧 src / launch / tests / 服务配置零改动。
**本任务未运行任何 ROS / DDS / 真实 UDP / HTTP**（含心跳——主代理之后执行）；
全部验证 = AST + 纯内存自测（fake socket / fake clock / fake http / 注入 worker）。
协议解析依据：**仅在线官方文档**（下述 alidocs URL）；本任务未读取旧工程
vendor_text 文档，文档中不得声称其它来源。

官方参考 URL（记录）：
- https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/gwva2dxOW4Kb6v3btk13Eplv8bkz3BRL
- https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/Gl6Pm2Db8D30AMZ0Se3l4yB5JxLq0Ee4

---

## 1. 实时链路（本轮接起来的缺口）

```
        只读 UDP：AOS 10.21.31.103:30000（telemetry.py）
        出站仅 Type=100 Cmd=100 心跳 1Hz（send 非阻塞）；收 Type=1002 Cmd=6 BasicStatus
        fatal 事件（decode_error/recv_error/heartbeat_error/comm_fault/错误来源）
          → Controller 立即 guard.disarm；live armed_once → 立即 session lock

  /new_chase/scan (SensorDataQoS) ── inputs.check_scan（frame/stamp 精确 int/
        窗口[-0.1,0.5]/元数据/包络屏蔽/停止区单帧立停；recv_age 门控 + 未来戳钳 0）
  /new_chase/cmd_vel_raw (Twist depth1 reliable) ── 三分量安全转换 → command
  http://127.0.0.1:8080/api/status ── 独立有界 worker 线程 GET（≤10Hz，timeout 0.1，
        主循环只读不可变缓存+完成时刻；仅新结果 update_tracking，绝不重复刷新）
                     │
                     ▼
   MotionGuard(output_enabled=True) ──GuardedOutput──▶ NavCmdSender
   dryrun: 绝不 arm（evaluate 显示真实阻断原因）       dryrun enabled=False → 无 /NAV_CMD
   live: 仅经 /new_chase/arm 显式人工 arm              live enabled=True → 懒创建
```

- 诊断：每 1s 向 `/new_chase/motion_guard_status` 发布 + stdout 打印 JSON
  （mode / pub_confirmed / decision / session / status / scan / web / raw /
  sender / ownership / live_block_reason）。
- arm 服务：`/new_chase/arm`（std_srvs/SetBool）。dryrun `data=true` 永拒；
  **绝不自动 arm / 绝不自动恢复**。
- **exit 0 ≠ 可运动**：exit 0 只表示本进程有界运行正常结束（dryrun 计时到/SIGINT）。

## 2. main 的时序与调度（修复1 后的合同）

1. `parse_runtime_args`（fail-closed：未知项/remap/位置参数 → exit 2；live 三旗
   + 禁 --no-telemetry + seconds 20..120；dry 10..120）；
2. `check_process_env(live)`：进程环境 fail-closed 校验（先于 ROS import）：
   - `FASTRTPS_DEFAULT_PROFILES_FILE` 必须精确等于批准的
     `config/diagnostic_udp_only.xml`，并用 stdlib ElementTree 结构校验：
     恰 1 个 UDPv4 transport（**SHM 一律拒绝，防 500MB 段**）、
     interfaceWhiteList 精确 {127.0.0.1, 10.21.31.104}、恰 1 个 default
     participant、userTransports 恰 [udp_transport]、useBuiltinTransports=false；
   - `RMW_IMPLEMENTATION` 必须 `rmw_fastrtps_cpp`；
   - `ROS_DOMAIN_ID`：live 必须 `0`；dryrun 允许 `0` 或 `47`（隔离测试）；
3. `rclpy.init(args=[])`（显式空参数，无任何 remap）→ Node（专属随机名
   `motion_runtime_<8hex>`，use_global_arguments=False 等，见模块 docstring）；
   **live 启动前对"未发现态"（nav==0 且 raw==0）做 ≤3s 有界发现等待**
   （每 50ms sleep 再查询，`_wait_live_discovery`；通过=nav0/raw1；任何
   nav>0 / raw>1 / probe 异常 / 非法类型立即拒绝不等待；3s 后仍 raw==0 拒绝；
   SIGINT / rclpy 停止 → 正常 finally 退出；dryrun 默认保持原单次核查不等待；
   不创建 NAV publisher、不 arm、不动服务、不改 mode）；
   **⚠️ 有界等待不等于运动授权：ownership/独占控制仍必须人工确认**；
   另：healthy 时 `live_block_reason` 会清除（仅未 session_locked 时；locked 后
   历史 reason 永久保留作证据）——避免首帧 raw=0→1 过程的旧 fatal 长期滞留；
4. 主循环名义 **20Hz deadline 调度**（RUN_PERIOD=0.05）：**批量 spin_once(0)
   非阻塞 drain**（第2轮修复：此前每周期仅 1 次 spin_once 只处理一个回调，
   scan 10Hz + raw ≈20Hz 时每秒最多约 20 个回调被处理、主代理 dryrun 实测
   scan 仅 ~2Hz；现每周期最多 SPIN_BATCH_MAX=16 次 spin_once(0)、累计
   SPIN_BATCH_BUDGET=5ms 内才开始下一次；20Hz 门控节拍与全部门控阈值、
   输出授权、遮罩均不变）→ 遥测 poll（非阻塞）→ worker 缓存消费 →
   图核查（≤0.1s）→ **tick(time.monotonic())（输入收完后的新鲜时刻）** →
   deadline sleep；工作超时则顺延下一周期（不忙转、不追发），故不声称
   "任何情况下恒 20Hz"；**已知局限（如实声明）**：rclpy 合同下单个回调执行
   不可抢占，单个长回调（如单帧 scan 处理过慢）可能超出 5ms 预算，只能使
   下一周期 deadline 顺延，不存在可抢占中断；无新线程、不使用 rclpy 内部
   私有 API、无 busyloop/无限 drain；**该修复未做吞吐实测**；
5. finally：live 曾激活则先 disarm+尝试一次零速 → stop+join web worker（有限超时）→
   sender.close → telemetry.close → executor/node/rclpy 回收 → **先按 cleanup
   错误定 exit 2，再打印与最终退出码一致的 summary**。

## 3. 心跳/遥测合同（telemetry.py）

- 出站唯一帧：心跳 Type=100 Cmd=100（默认 1s 间隔；失败同样计入间隔，序列号
  0..65535 循环）；服务端 2s 无请求停推——dryrun 里发心跳是**只读订阅动作**
  （注册 BasicStatus 推送源），已在协议文档如实说明，不等于"绝对无网络"；
- 单次解析分类：malformed → `decode_error`；ACK ErrorCode=0 → `ack_ok`
  （不刷新状态）；ACK 非 0 → `comm_fault`；其它 Type/Command → `ignored`
  （忽略不刷新）；错误来源端点 → `decode_error`（保守按输入错误处置）；
- 只有有效 status 刷新 `last_status/status_received_at`（单调秒）；无有效数据
  绝不更新任何时间戳；报文 Time 无认证签名，不作为任何新鲜度依据。

## 4. 所有权（ownership）合同

- 图核查频率 **≤0.1s**（不是 1s、不是只记录）：
  `nav_foreign_publishers != 0` 或 `raw_publishers != 1` 或 probe error →
  **立即 guard.disarm**；live 且 armed_once → **立即 session lock**；
- `request_arm` 严格类型判定：`type(nav_foreign) is int and == 0`，
  `type(raw_publishers) is int and == 1`；unknown / probe error → 拒绝并锁死
  （绝不"不是 int>0 就放行"）；
- 自身排除：**禁止纯同名排除**。节点使用专属随机名 `motion_runtime_<8hex>`；
  仅当 `pub_confirmed`（sender 曾成功构建并发布过，send 失败不算）且图中出现
  该专属名的 endpoint 时才把 /NAV_CMD 总数减 1；同名碰撞概率与
  "DDS 图身份非加密认证"的局限在此如实记录；
- live 前 ROS 图核查**不证明 planner 已停**：操作员必须现场确认 planner
  inactive、charge_manager idle、无他方发 /NAV_CMD、现场有人手急停。

## 5. live 会话（**本任务未执行，绝不因"只加 --live"而安全**）

- live 需同时：`--live --commissioned --exclusive-control-confirmed` 三旗 +
  遥测开启（--no-telemetry 禁止）+ `ROS_DOMAIN_ID==0` + 批准 UDP-only profile；
- Controller 构造即校验 `live == sender.enabled`（精确 bool）——dryrun 绝不允许
  sender enabled=True；
- `request_arm(true)`：检查 `sender.enabled` → 严格所有权判定 →
  `set_exclusive_control(True)`（每次显式请求=人工确认独占）→ `guard.arm`；
  **只允许首次**成功；deadline = 成功时刻 + 10s（单调）；
- 到期判定用 `>=`（恰好 10s 即锁定，不再发送）；人工 disarm **即刻 flush 一次
  零速**（output.tick 新鲜时刻）；发送异常同 tick 立即 session lock（不留
  下一拍窗口）；
- 首次任一输入失效 / 上述任一 fatal → 进程永久锁死，re-arm 拒绝（不刷新 10s）；
  重启进程 = 重新人为明确授权；所有 fatal 后不自动再 arm；
- live shutdown：disarm + 一次性零速补发；**send 失败无法保证零速送达**，
  兜底只能靠机端看门狗 / 现场急停（如实声明）。

## 6. dryrun 域 47 测试约定

- 隔离测试允许 `ROS_DOMAIN_ID=47` + `--no-telemetry`（**不触 AOS 30000**）；
- domain 47 下 HTTP GET 默认仍会访问真实 Web（仅只读 GET，可接受）；
  正式 tester 用注入（`Controller.poll_web` / `fetch_web_status(urlopen=…)` /
  `WebStatusWorker(fetch=注入)`），不发起真实 HTTP；
- dryrun 真实网络（域 0 + 遥测）由主代理执行。

## 7. 验收接口（给 tester；全部纯内存、无 ROS / DDS / 真网络）

```python
from motion_control.telemetry import (
    encode_heartbeat, decode_apdu, extract_basic_status, decode_basic_status,
    decode_ack_error, TelemetryClient)                 # sock/monotonic 可注入
from motion_control.inputs import (
    check_scan, parse_web_status, fetch_web_status, target_generation)
from motion_control.runtime import (
    Controller, parse_runtime_args, check_process_env,
    validate_approved_profile_xml, WebStatusWorker, LIVE_SESSION_SECONDS)
```

- `TelemetryClient.poll(now) -> events`；fake socket 需实现
  `send/recvfrom/setblocking/close`；
- `check_scan(scan, now_wall, recv_age=None)` → `{valid, clear, source_age,
  reason, valid_count, point_count, envelope_skipped}`（绝不抛异常）；
- `Controller`：`on_scan/on_raw/poll_web/consume_web/poll_telemetry/
  check_ownership_fatal/request_arm/tick/disarm_for_input_error/shutdown_output/
  snapshot`；`pub_confirmed`/`send_calls`/`send_errors` 公开计数；
- `WebStatusWorker(fetch=注入)`：`start/latest/stop`（stop 返回 joined）；
- `check_process_env(live)` / `validate_approved_profile_xml(path)`：
  可用真实批准文件（UDP-only 通过；SHM XML 拒绝）做只读校验测试；
- 禁止事项（tester 同样遵守）：不 rclpy.init、不起 DDS、不建真实 /NAV_CMD、
  不向 10.21.31.103:30000 发真实 UDP、不改 core/nav_sender 私有与 TTL。

## 8. 边界声明

- 未运行 ROS/DDS/真实网络；未宣称任何实机行为；四 preview 服务原样未动；
- 未读旧工程（vendor_text 等）做任何复用；未安装依赖；无 git 提交；
  无 `__pycache__`（-B + PYTHONDONTWRITEBYTECODE）；
- 本节点无自动 systemctl/shell/popen/ssh；无外部 HTTP（仅 GET 本机 8080）；
- 适配层不修改 core 的 TTL/语义；本包不构成运动验收；真实全链路验证由主代理
  与 tester 后续执行。
