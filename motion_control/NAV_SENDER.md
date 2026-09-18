# NAV_SENDER — /NAV_CMD 发送边界适配器（默认禁用，**不能直接试走**）

`nav_sender.py` 提供 `NavCmdSender`，本文档说明其合同与 `GuardedOutput(guard, sender)` 组合方式。本轮仅新增 `nav_sender.py` 与本文件，工程其余部分未改动。

## ⚠️ 最重要的警告（先读）

- **本模块不构成已接真实状态的完整运行节点**。当前没有 live 服务、
  没有实时 BasicStatus adapter、没有目标状态 adapter。`nav_sender.py`
  只是把 `Velocity` 转成 `NavCmd` 消息并发到固定 `/NAV_CMD` 的发送边界，
  **绝不能直接试走**。
- 调用方（未来现场集成）自行负责全部安全链路：
  - `MotionGuard` 的所有实时输入：`update_status`（真实 BasicStatus）、
    `update_scan`（真实避障判定）、`update_tracking`、`update_command`、
    `set_exclusive_control`；
  - **20Hz 调度**（`GuardedOutput.tick(now)` 的定时驱动）；
  - 异常停止（急停、看门狗兜底——网络断开时零速**无法保证送达**）。
- 本代码**没有**任何自动切模式 / 起立 / planner stop / arm 逻辑；
  单个 sender 也**不会**自动 arm。arm 必须由调用方在验证 planner 已停、
  独占控制人工确认后显式执行。
- 用户原目标「10 秒试跟随」**暂未执行**；「先接入但暂不试走」是用户明确选择。
- 本文档**不提供**「复制即可开机运动」的命令，只提供 API 与纯内存模拟测试说明。

## 接口（给 tester / 集成方）

```python
from motion_control.core import MotionGuard, GuardedOutput, Velocity
from motion_control.nav_sender import NavCmdSender

sender = NavCmdSender(node=None, enabled=False, message_type=None)
```

- `enabled` 必须**精确 bool**（`True`/`False`，不接受 `1`/`"true"`），
  默认 `False`。
- `message_type=None` 时**惰性** `from drdds.msg import NavCmd`（首次启用发送才 import）；
  测试可传替身消息类。
- topic **固定 `/NAV_CMD`**，无 topic 参数，不支持也不接受 remap 到其他路径。
- `node`：enabled=True 时必须传调用方自有 rclpy Node（见下）。
- `sender.close()`：幂等；只销毁 sender 自己的 publisher，**不 shutdown node**，
  **不补发零速**（停止零速由 `GuardedOutput` 在销毁 sender 前的最后一次
  blocked tick 负责）。close 后 enabled sender 的 `__call__` 拒绝发送。

### enabled=False（默认，本轮状态）

- **不 import ROS 消息、不访问 node、不创建 publisher、不发送任何消息
  （连零速都不发）**；`__call__(velocity)` 恒返回 `False`。
- 此时可传 `node=None`；传了 node 也不会被访问。

### enabled=True（未来现场集成授权选项，本轮仅替身 node 测试）

- 必须传 `node`：**独立调用方 Node，建议参数隔离**：

  ```python
  node = rclpy.create_node(
      "my_nav_sender_host",
      use_global_arguments=False,  # 不吃全局 argv，杜绝 --remap 注入
      cli_args=[],                 # 显式空参数
  )
  ```

  这是防 remap 的主防线（文档合同，代码不重复校验 argv）。
- publisher 懒创建：**首次 enabled 发送时**才 `node.create_publisher(
  msg_type, "/NAV_CMD", depth=1)`（reliable / volatile 默认）。构造时不创建。
- 创建后复核 `publisher.topic_name`：若 `!= "/NAV_CMD"` **立即
  best-effort `destroy_publisher` 并 raise**，绝不 publish——不存在「创建到
  错误 topic」之后还发出的路径。注意措辞：destroy 是 **best-effort，
  不夸大保证已销毁**（销毁失败不掩盖 topic 不符这一主错误）。

### 创建期异常一律锁存（修复 1）

publisher 创建路径上的**任何**异常——惰性 `from drdds.msg import NavCmd`
失败、msg 替身工厂抛错、`node.create_publisher` 抛错（含 `ValueError`，
不仅限于 `RuntimeError`）——都**锁存 fault 并把原异常原样 raise**。
之后所有 `__call__` 以 `RuntimeError（故障锁存）` 拒绝，**不再再次调用**
`create_publisher` / import，直到重新实例化。topic 不符仍保持
`topic_mismatch` 原因不变。
- `header.frame_id`：uint64 从 0 递增、到顶回绕；`header.stamp` 取
  `node.get_clock().now().to_msg()`（节点时钟，**非墙钟**；入参不携带 stamp，
  发送时才写入）。

### `__call__(velocity: Velocity) -> bool`

- `velocity` 必须是 `core.Velocity` 实例；分量必须**精确** `int`/`float`
  （拒绝 `bool`）、有限（拒绝 NaN / Inf / 溢出）。非法 → `raise ValueError`
  **并锁存 fault**，之后所有调用拒绝，直到重新实例化。
- 发送前**独立再次限幅**（不依赖 core 私有 `_limit`）：
  `x ∈ [0, 0.3]`、`y = 0`、`yaw ∈ [-0.6, 0.6]`；`|x| < 0.2 → 0`、
  `|yaw| < 0.5 → 0`。死区内置零，**绝不向上抬**。
- 写入 `msg.data.x_vel / y_vel / yaw_vel`（SI 单位）。
- 返回 `True` 表示 publish 调用成功；**最后一条网络包不保证送达**（无确认）。
- `publish` 失败：原异常 raise 且 fault 锁存，后续调用拒绝，不自动复活。

### 与 GuardedOutput 组合（推荐）

```python
guard = MotionGuard(output_enabled=False)   # 默认禁用，绝不输出
sender = NavCmdSender(node=my_node, enabled=False)  # 本轮替身状态
output = GuardedOutput(guard, sender)       # guard 放行才 send
d = output.tick(now)                        # now 由调用方单调钟提供（20Hz 驱动）
sender.close()                              # 只关 publisher，不动 node
```

- `GuardedOutput` 的零速补发 / send 异常 disarm 语义见 `README.md`；
  sender 的 fault 锁存在其之上独立生效。
- **单 sender 不自动 arm**；不要直接绕过 core 在运行系统启用。

## 故障锁存汇总

| 触发 | fault 值 | 后续 |
|---|---|---|
| 输入非法（ValueError） | `发送时锁存` | 所有调用拒绝，直至重新实例化 |
| publisher 创建失败（import / msg 工厂 / create_publisher，任何 Exception，含 ValueError） | `publisher_failed: <ExcName>: …` | 同上，后续调用拒绝且不再触 create |
| 实际 topic ≠ /NAV_CMD | `topic_mismatch: actual=…` | 同上（best-effort 销毁，不保证已销毁） |
| publish 异常 | `publish_failed: …` | 同上，不自动复活 |

## 本轮自测边界（tester 请注意）

- 本轮仅做过 AST 校验与（计划中的）FakeNode/FakeMsg 纯内存替身验证；
  **未** `rclpy.init`、未起真实 DDS、未用 ROS CLI、未 import 旧工程代码。
- 正式测试文件由 tester 编写，本包不含测试代码。
