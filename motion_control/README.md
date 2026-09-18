# motion_control — M20Pro 运动放行门控（MOTION-GUARD-CORE）

纯 Python 逻辑核心（Python 3.8 标准库，无 ROS / 无 socket / 无网络认证）。
默认 `output_enabled=False`：绝不输出，也绝不调用发送回调。真实发送端尚未接入，
本包不是实机验收，不能直接启动让机器人运动。

## 背景与边界

- 协议本身不要求门控，但自动跟随场景加最低保护是合理的。
- NavCmd / Cmd25 仅在 `ControlUsageMode == 1` 时生效。
- 本 core 是离线逻辑 + 未来发送边界适配合同：未连接实时 BasicStatus、
  实际 ScanReader、target 输出；计划不改动现有 preview node 接口（旧预算不再动）。
- **不宣称做了网络安全认证**：UDP 来源核查（IP/端口/签名）是未来 adapter 的责任，
  本 core 只做内容级校验。
- `clear`（路径无障碍）由上游实际避障/扫描校验给出；core 本身不做几何计算。
- `set_exclusive_control(True)` 是人工确认的**软条件**，不等于主动证明 planner
  已停止；真实接入方必须自行验证 planner / 充电占用与指令来源，不能仅靠这个 bool。

## 状态字段（只承诺这些官方字段；多余字段忽略，StatusCode 等未列不强制）

| 字段 | 必须值 |
|---|---|
| MotionState | 17 |
| ControlUsageMode | 1 |
| Direction | 0 |
| HES | 0 |
| Charge | 0 |
| Sleep | 0 |
| Gait | 4097 |

必须精确 `int`（bool 不算）：缺字段 / 错类型 / 值不符 → 立即 `status_invalid`
并解除 arm（不忽略 bad 保留旧 good）。反例：缺 `ControlUsageMode` 或
`MotionState != 17` 均不放行。

## API（时间均为调用方提供的单调钟秒 float；不用墙钟判 TTL）

```python
@dataclass(frozen=True)
class Velocity:            # SI 单位，原始值；不做归一化
    x: float = 0.0; y: float = 0.0; yaw: float = 0.0

@dataclass(frozen=True)
class Decision:
    velocity: Velocity     # 放行时为限幅后指令，否则全零
    armed: bool
    allowed: bool
    reason: str            # 稳定 lowercase 标识

class MotionGuard(output_enabled: bool = False)   # 必须精确 bool，不接受字符串
    update_status(status: dict, received_at: float)
    update_scan(valid: bool, clear: bool, source_age: float, received_at: float)
    update_tracking(valid: bool, received_at: float)
    update_command(command: Velocity, received_at: float)
    set_exclusive_control(confirmed: bool)
    arm(now: float) -> Decision
    evaluate(now: float) -> Decision
    disarm() -> None

class GuardedOutput(guard: MotionGuard, send: Callable[[Velocity], None])
    tick(now: float) -> Decision
```

## TTL（秒，单调钟）

| 项 | 值 |
|---|---|
| command | 0.3 |
| status | 0.5 |
| scan（source_age + 停留时间） | ≤ 0.5，未来最多 0.1 |
| tracking | 0.5 |

`source_age ∈ [-0.1, 0.5]`；evaluate 到当前时重新计算
`source_age_at_receipt + (now - received_at) ≤ 0.5`，防止重复缓存续命。
`received_at` / `now` 为未来时间会拒绝对应 stale / `clock_invalid`；
`timestamps == 0` 是合法的单调钟测试值。
任何 `now` 非法（非 finite / bool / 负数 / 相比上次 evaluate/arm 回拨）→
`clock_invalid` 并解除 arm。

## 放行逻辑

- `arm()` 需同时满足：`output_enabled=True` + `exclusive=True` + status/scan/
  tracking/command 全部有效且新鲜。任一不满足 → `armed=False` 拒绝（不排队）。
- 任一 update_* 收到 bad / 过期 → 立即解除 arm。恢复数据后 evaluate 仍为零，
  必须再次显式 `arm()`；最新输入符合也不会自动 arm。
- 限幅（先 clamp 再死区，绝不提高速度，不取绝对值代替符号）：
  x ∈ [0, 0.3]（禁后退）；y = 0（basic 步态 y 最小 0.35 > 上限 0.3）；
  yaw ∈ [-0.6, 0.6]；|x|<0.2 → 0；|yaw|<0.5 → 0。

## reason 一览（稳定 lowercase 标识）

`ready / disarmed / output_disabled / status_missing / status_invalid /
status_stale / scan_missing / scan_invalid / scan_stale / tracking_missing /
tracking_invalid / tracking_stale / command_missing / command_invalid /
command_stale / ownership_missing / clock_invalid`

（`ownership_missing` 预留：当前 `set_exclusive_control(False)` 拒绝 arm 走 `disarmed`。）

## GuardedOutput 发送边界

- `output_enabled=False` 或从未放行过：**绝不调用 send**（连零速都不发），
  避免干扰既有控制链路。
- 曾放行后转入 blocked（失败 / 显式 disarm / 超时）：只补发**一次**零速，
  之后不再重复发零速垃圾。
- send 异常：`guard.disarm()`（同时清除 exclusive，阻止自动恢复），并把异常
  原样传播给调用方，绝不吞错伪成功。断网时无法保证零速真实到达机器——
  只能靠机端看门狗 / 现场急停兜底。
- 不创建任何真实 transport（无 ROS topic / 无 socket）。

## 测试（tester 待写 test_motion_guard.py，本包不含测试代码）

```bash
cd /home/user/new_chase/motion_control
python3 -B -m pytest test_motion_guard.py   # 或 python3 -B -m unittest discover
```

