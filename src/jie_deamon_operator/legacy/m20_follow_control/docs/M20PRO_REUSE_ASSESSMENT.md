# M20 Pro / `jie_deamon` Reuse Assessment

- Assessment date: 2026-09-03
- `jie_deamon` commit: `9403185c0cf99a0b415ddd4f3b4215b44dd0809c`
- M20 guide reviewed: Software Development Guide V1.2.1, updated 2026-05-18
- Verdict: The application structure is reusable, but the proposal is not safe to deploy unchanged. The M20 bridge, failure handling, network control, action semantics, and sensor-frame assumptions must be changed first.

## Verified Facts

`jie_deamon` subscribes to `/scan` and publishes `/cmd_vel` and `/d1_cmd`. Its LiDAR launch is a placeholder and D1 support comes from an external `d1_bringup/d1_core` package:

- [ROS interfaces](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/robot_nexus.cpp#L60-L66)
- [Placeholder LiDAR and D1 launch](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/launch/start.launch.py#L51-L63)
- [External D1 core](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/launch/d1_bringup.launch.py#L6-L24)

The M20 guide defines UDP at `10.21.31.103:30000`, TCP at `:30001`, heartbeat, usage mode, motion state, gait, and normalized axis commands. `X/Y/Yaw` can represent `Twist.linear.x`, `Twist.linear.y`, and `Twist.angular.z`. The official Android SDK uses the same UDP/JSON control model and maps a joystick to `AxisData(x, y, yaw)`:

- [M20 Android SDK](https://github.com/DeepRoboticsLab/m20-android-sdk/blob/main/README.md)
- [Robot SDK API](https://github.com/DeepRoboticsLab/m20-android-sdk/blob/main/docs/sdk-docs/robot-sdk-api-reference.md)
- [Controller SDK API](https://github.com/DeepRoboticsLab/m20-android-sdk/blob/main/docs/sdk-docs/controller-sdk-api-reference.md)

The on-robot ROS distribution is Foxy. Official deployment instructions source `/opt/ros/foxy` and `/opt/robot/scripts/setup_ros2.sh`:

- [M20 SDK deployment](https://github.com/DeepRoboticsLab/sdk_deploy/blob/main/src/M20_sdk_deploy/README.md#L97-L127)
- [M20 Lightning-LM deployment](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics#62-preparation)

The guide's LiDAR interface is the single 10 Hz `sensor_msgs/msg/PointCloud2` topic `/LIDAR/POINTS`. It requires administrator privileges and depends on `multicast-relay.service`. Use that topic first; do not assume the application must merge separate front and rear raw topics:

- [Official M20 point-cloud checks](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics#612-point-cloud-permissions)
- [ROS pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)

The claim that public `drdds` contains only joint-level messages is outdated. As of 2026-09-03, the official public message repository includes `Gait`, `MotionInfo`, `MotionState`, and `NavCmd`, with Foxy/Humble/Jazzy support:

- [Official message files](https://github.com/DeepRoboticsLab/deep-robotics-msg/tree/main/msg)
- [Official message repository](https://github.com/DeepRoboticsLab/deep-robotics-msg)

Do not overwrite the robot's installed `drdds`; the official M20 SDK deployment guide warns against rebuilding it on the robot.

## Corrections To The Proposal

1. Prefer the M20 Pro GOS development host (`10.21.31.104`) for the application. First verify that GOS can subscribe to `/LIDAR/POINTS` and reach the AOS UDP endpoint. Do not place Web, tracking, and Android services on AOS by default.
2. Treat `/LIDAR/POINTS` as the first sensor source. Only build a front/rear fusion node if live topic inspection proves it is needed.
3. `jie_deamon` sends `standup`, not `stand`, and sends `passive` five seconds after `liedown`. The M20 adapter needs a feedback-driven state machine, not a direct string-to-integer table.
4. Foxy compatibility is likely but not build-tested. Upstream requires Humble or later. OpenCV is a mandatory compile-time dependency even when visualization is disabled.
5. Use one persistent UDP socket. The M20 protocol requires axis commands within a two-second window to come from the same client. Parse every response and expose `ErrorCode`; do not fire-and-forget JSON packets.
6. `/NAV_CMD` is a possible physical-units alternative, but the guide says it requires navigation mode and can conflict with planner and charging services. For an isolated follower, regular mode plus UDP axis control remains the preferred first path.

References:

- [`standup` and `liedown`](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/web/app.js#L720-L742)
- [`liedown` followed by delayed `passive`](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/robot_nexus.cpp#L102-L118)
- [Humble-or-newer requirement](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/README.md#L51-L58)
- [C++17 and OpenCV dependency](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/CMakeLists.txt#L1-L41)

## Blocking Safety And Security Defects

### WebSocket command injection

The unauthenticated `Sec-WebSocket-Key` header is concatenated into a shell command and executed by `popen()`. The server listens on all interfaces. If the node runs as root, this is a potential remote root command-execution path. Replace the handwritten handshake with a maintained WebSocket library or in-process SHA-1/Base64 before enabling Web:

- [Shell construction and `popen`](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/web_comm.cpp#L224-L237)
- [All-interface listener](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/web_comm.cpp#L125-L153)

### Unauthenticated motion control

HTTP, WebSocket, and Android UDP bind to all interfaces. WebSocket clients can switch modes and send arbitrary velocity/action commands; UDP clients can change target and motion-enable state. Disable these interfaces by default until authentication, source allowlisting, interface binding, and firewall rules are added:

- [WebSocket commands](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/web_comm.cpp#L195-L221)
- [Android UDP commands](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/android_comm.cpp#L53-L100)

### No dead-man timeout

Direct control republishes the last command every 100 ms forever, including after client disconnect. Add a 200-300 ms timeout in `jie_deamon` and a separate timeout in the M20 bridge:

- [Repeated direct command](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/include/direct_control.hpp#L28-L63)
- [10 Hz timer](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/src/robot_nexus.cpp#L130-L134)

### Stale target motion

If no target points are found, the old target remains valid and is still used to calculate motion. An empty scan returns without publishing zero. Add target validity, consecutive-miss counting, scan freshness, and immediate zero output on loss:

- [Empty scan return](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/include/lidar_tracker.hpp#L77-L93)
- [Target updated only when found](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/include/lidar_tracker.hpp#L198-L213)
- [Velocity calculated from old target](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/include/lidar_tracker.hpp#L215-L236)

### Sensor-frame assumption

The tracker negates both standard LaserScan coordinates, equivalent to a 180-degree rotation. This may compensate for the original D1 sensor mounting. Inspect `/LIDAR/POINTS.header.frame_id`, TF, and axis signs before deciding where to transform data:

- [Coordinate negation](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/include/lidar_tracker.hpp#L112-L119)

### Unsafe initial parameters

Defaults are 0.4 m following distance, 0.2 m emergency distance, 0.25 m slowdown distance, and 1.0 m/s maximum linear speed. They are unsuitable as first-test values for an approximately 820 mm by 430 mm robot:

- [Current constants](https://github.com/6-robot/jie_deamon/blob/9403185c0cf99a0b415ddd4f3b4215b44dd0809c/include/common_types.hpp#L15-L37)
- [Official M20 specifications](https://www.deeprobotics.cn/en/index/lynx.html)

Engineering starting point: 0.15-0.25 m/s linear speed, 0.3 rad/s yaw speed, 1.0-1.5 m following distance, at least 1.0 m slowdown distance, plus acceleration and jerk limiting. Final values require braking and terrain tests.

## Recommended Architecture

```text
M20 Pro GOS (preferred, 10.21.31.104)
  m20_io_bridge
    /LIDAR/POINTS -> pointcloud_to_laserscan -> /scan
    /cmd_vel -> clamp/slew-limit/watchdog -> UDP axis -> AOS:30000
    /m20/action -> feedback-driven UDP motion/gait state machine
    heartbeat + response/error parsing + HES/mode/state/charge gating

  jie_deamon_m20
    fixed target-loss handling
    inactive and motion-disabled by default
    Web/Android disabled by default until hardened
```

Bridge requirements:

1. Persistent single UDP socket and increasing message ID.
2. 20 Hz axis output; zero after 250 ms without fresh `/cmd_vel`.
3. Zero-command burst on stop, shutdown, exception, and network/sensor loss.
4. Nonzero output only when explicitly armed, in regular mode, in confirmed RL-control state, not hard-stopped, not charging, and without critical robot error.
5. Parse every response and publish diagnostics.
6. Clamp `Twist` to configured safe physical limits before converting to `[-1,1]`.
7. Never stand up or enable movement automatically at process startup.

## Live-Robot Verification Sequence

1. Confirm system V1.1.8, host IPs, architecture, disk/memory, Foxy, installed `drdds`, and `ROS_DOMAIN_ID`.
2. On GOS, perform read-only checks for `/LIDAR/POINTS`, `/IMU`, `/MOTION_INFO`, and `/tf`.
3. Inspect `multicast-relay`, `rl_deploy`, `planner`, `basic_server`, `charge_manager`, and `localization` without stopping anything.
4. Record a 30-60 second rosbag and validate point-cloud projection and TF offline.
5. Run heartbeat/status reception only; send no motion command.
6. Send zero axis commands and validate response codes and single-client ownership.
7. In an open flat area with an operator holding the physical emergency stop, test tiny X, Y, and Yaw commands one axis at a time.
8. Prove stale `/cmd_vel`, process exit, network loss, LiDAR loss, and target loss all produce zero.
9. Enable following only after those tests, with Web and Android disabled.
10. Add systemd only after failure testing; startup must remain disarmed.

## Final Decision

Reusable: ROS application boundary, tracking-code structure, shared state, and visualization concepts.

Must be replaced or substantially changed: D1 action semantics, robot driver, LiDAR frame assumptions, control lifecycle, network servers, safety parameters, and launch behavior.

Treat this as a controlled refactor of `jie_deamon` into an M20 Pro application, not a two-launch-file swap.
