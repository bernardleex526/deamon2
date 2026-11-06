# M20 Pro Follow Control

This package ports the target-following structure from `6-robot/jie_deamon`
to the M20 Pro sensor and body-control contracts. It deliberately omits the
upstream unauthenticated HTTP, handwritten WebSocket, and Android UDP control
servers.

## Runtime graph

```text
/LIDAR/POINTS (sensor_msgs/PointCloud2)
  -> m20_follow_node
  -> /m20_follow/cmd_vel (geometry_msgs/Twist)
  -> m20_udp_bridge
  -> M20 APDU/JSON UDP 10.21.31.103:30000
```

Both motion gates default to `false`. Starting the launch file sends heartbeat
messages for status discovery but does not send axis, gait, or motion-state
commands unless `motion_output_enabled:=true` is explicitly supplied.

## Offline build and tests

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select m20_follow_control
colcon test --packages-select m20_follow_control
colcon test-result --verbose
```

## Read-only launch

```bash
ros2 launch m20_follow_control m20_follow.launch.py
```

Observe `/follow/status` and `/m20/bridge_status`. The bridge opens one
persistent UDP socket so heartbeat, status, and later axis commands use the
same client endpoint required by the M20 protocol.

## Controlled live sequence

Do not combine these steps into an automatic startup script.

1. Confirm `/LIDAR/POINTS` has live samples and validate its frame orientation.
2. Start with both launch gates false and confirm BasicStatus reception.
3. Start again with `motion_output_enabled:=true`; the bridge still sends only
   zero axis commands because it remains disarmed.
4. Publish `standup` to `/m20/action` and wait until bridge status reports
   usage mode 0 and motion state 17.
5. Publish `true` to `/m20/arm`.
6. Publish `true` to `/follow/enable` only in a clear test area with the
   physical emergency stop held by an operator.

The bridge returns to zero output when `/m20_follow/cmd_vel` is stale for 250 ms, robot
status is stale, HES is active, charging is active, usage mode is not 0,
motion state is not 17, the configured forward direction is reversed, an
action is pending, or the bridge is disarmed.

## Sensor transport boundary

The prior `m20_orignal` live adaptation found that ROS graph visibility did
not guarantee `/LIDAR/POINTS` samples. On AOS, keep vendor DrDDS in the
standalone `m20_drdds_receiver` process. Do not load vendor DrDDS and ROS 2
FastDDS into this process. The follower accepts the receiver's existing wire
format through a Unix socket:

```bash
# Terminal 1, vendor-only helper on AOS
ros2 run m20_slam_navigation m20_drdds_receiver -- \
  --lidar-topic /LIDAR/POINTS \
  --lidar-socket /tmp/m20_follow_lidar.sock \
  --no-imu

# Terminal 2, ROS 2 follower and UDP bridge
ros2 launch m20_follow_control m20_follow.launch.py input_mode:=socket
```

Use `input_mode:=ros` for bag replay, workstation tests, or a robot host that
actually receives standard ROS `PointCloud2` samples.
