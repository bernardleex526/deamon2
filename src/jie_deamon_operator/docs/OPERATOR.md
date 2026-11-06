# M20 手机与一键操作

本版本保留 deamon2 的点云转二维扫描与 D1 跟随算法，未修改跟随数学逻辑。
旧 deamon 三维实现保存在 `legacy/m20_follow_control`，由 `COLCON_IGNORE` 排除。
不要同时启动旧包、原厂导航或另一套速度控制程序。

## 一次性准备

在 GOS 的 `/home/user/m20_ws/src/jie_deamon` 放置本分支，使用原厂 Foxy 环境编译：

```bash
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20_ws
colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

依赖与基线相同：ROS 2、OpenCV、pointcloud_to_laserscan、Python 3。新网页仅使用
Python 标准库；无需 npm。确保 GOS 到 NOS 的 SSH 主机指纹已核实，GOS 操作用户已有
NOS SSH 密钥登录。脚本不接受明文密码，也不关闭 SSH 主机校验。

NOS 转发若需要脚本自动启动，管理员可用 `visudo` 为该用户授权**仅**执行
`systemctl start multicast-relay.service`（使用本机 `command -v systemctl` 的绝对路径）。
也可以由操作者先启动该服务；脚本看到它已运行后不会再调用 sudo。不要授权任意 shell。
这不设置开机自启，不创建或修改厂商转发服务。

## 日常演练：在 GOS 执行一条命令

```bash
cd /home/user/m20_ws/src/jie_deamon
bash scripts/m20ctl preview
```

该命令申请本机 sudo、加载已验证的 root DDS 环境、检查重复节点、通过 SSH 检查/启动
NOS 转发、用 12 秒探测确认点云后启动网页。失败会退出并显示具体错误。AOS 无需登录。
路径或 NOS 地址不同可设置 `M20_WS`、`M20_NOS_HOST`。不要在同一工作区同时保留可被
colcon 发现的另一个 `jie_deamon` 包。

另一个 GOS 终端查看口令：

```bash
bash scripts/m20ctl token
```

手机连入可访问 GOS 的可信局域网，打开 `http://<GOS可达IP>:8080`，输入口令。
HTTP 不加密，口令只用于隔离局域网访问，不能把该端口暴露到互联网；远程访问应使用
受控 VPN/TLS 入口。口令文件只对 root 可读，不存进 Git 或网页本地存储。

页面显示雷达图、运行模式、状态拦截原因、原始/放行速度。点击目标后选“开始跟随”，或
选“开始手动控制”并按住方向键。松开方向键发送零速度；切换模式先停止。
演练中 guard 始终不使能，放行速度始终为零，不伪造机器人状态，但可查看原始算法输出。
当前正前方目标产生较大横移的历史问题没有在本次接口修改中修复，必须继续 dry-run 排查。

```bash
bash scripts/m20ctl check    # 只检查，不启动转发或控制
bash scripts/m20ctl status   # 查看已运行服务的状态
bash scripts/m20ctl stop     # 撤销控制与跟随；网页服务仍运行
```

`stop` 返回是软件请求已处理；确认实机静止后，在启动终端 Ctrl+C 结束全部 launch 进程。
进程崩溃/网络断开后的实体停车仍依赖原厂超时保护，必须实测。

## 实机入口（完成验收之后）

先修复/验收跟随方向与丢失目标行为，按原 SOP 完成原厂控制权移交、状态、步态、急停与
实际停车距离检查。脚本不替你停止 planner，也不证明原厂导航/充电/建图已经退出。
结束演练 launch，然后：

```bash
bash scripts/m20ctl live --commissioned
```

启动仅发送心跳和零速度，不自动运动。手机“开始”才会检查完整状态和新鲜无近障扫描、
取得唯一控制权并 arm。失败原因显示在页面。指令 0.3 秒过期，扫描 0.5 秒过期，操作连接
0.6 秒过期；状态 1.5 秒过期。故障恢复不自动重新开始。桥的 20 Hz 调度会增加至多一周期
的软件检测延迟；这些不是实测停车距离。

## Android 协议

Android 手机可直接使用网页。原生 App 需要修改为以下协议；**原 D1 App 不能原样连接**。
UDP 默认 GOS 8889，可通过 launch `android_port:=0` 关闭。响应发回请求的源端口。
HTTP 同一 JSON 请求使用 `/api/command`，口令放 `Authorization: Bearer <token>`；
UDP 把 `token` 放进 JSON。

```json
{"token":"<口令>","client":"app-session-随机唯一值","seq":1,"op":"start","mode":"direct"}
{"token":"<口令>","client":"app-session-随机唯一值","seq":2,"op":"velocity","velocity":[0.2,0,0]}
{"token":"<口令>","client":"app-session-随机唯一值","seq":3,"op":"stop"}
```

每个会话 `seq` 严格递增，建议 10 Hz 发送直控速度；跟随使用
`start, mode=follow, target=[x,y]` 后以 10 Hz 发 `op=heartbeat`。普通心跳不刷新旧直控速度。
每次必须检查 `ok`，拒绝会返回 `error`；丢包不能当成功，重试需新的序号。一次只能有
一个来源持有控制权；任何持口令的客户端可 stop。每次 App 重启创建新 client，断连后需
人工重新开始。不是后台自动重试 start。

M20 站立、趴下、passive、步态尚无本分支验证过的映射；`op=action` 返回不支持。
使用原厂端完成这些操作。原 D1 `web/` 资源仍在源码中供追溯，但 M20 不启动它的
HTTP/WebSocket、旧 Android 接收器或 D1 动作发布逻辑。

## ROS 与测试入口

新 `enable_web=true` 启动桥内的认证网关，而不是旧 C++ Web 服务。直接使用 launch 时需
指定 `operator_token_file`（至少 32 字符）或环境变量 `M20_OPERATOR_TOKEN_FILE`。
`operator_host`、`operator_port`、`android_port` 可配置；m20ctl status/stop 使用默认 8080。

`/robot_nexus/operator_target` 的 Float64MultiArray 为 `[session,x,y]`，session=0 关闭跟随。
`/robot_nexus/operator_velocity` 为 `[session,vx,vy,wz]`。session 是小于 2^53 的整数，桥仅
接受当前会话输出；`/m20/cmd_vel_raw` 继续用于诊断，不绕过操作控制权。ROS 网络本身必须
可信：此 HTTP 口令不是 DDS 访问控制。原 ROS 调试流程可显式 `enable_web:=false` 使用。

```bash
python3 -m unittest discover -s test -v
colcon test --packages-select jie_deamon --ctest-args -R test_m20_tracker --output-on-failure
python3 test/ros_m20_smoke.py
python3 test/ros_operator_smoke.py
```

ROS smoke 使用隔离 Domain 83、localhost 和合成输入，不接触机器人。Windows 单元测试
不替代 Foxy 编译、手机浏览器真机验证、Android App 联调或实体运动验收。
