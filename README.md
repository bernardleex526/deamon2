# deamon2 / `jie_deamon` for M20 Pro

**September 11, 2026 software status:** the M20 profile has bounded basic-gait
approach/turn control, latched target association, body-clearance APF, and
timestamp-aware tracking interlocks. See the detailed
[M20 fixes and dry-run SOP](docs/M20_FIXES_20260911.md).
Physical following is **not accepted**. Software tests and recorded-scan replay
do not replace supervised robot-side stop, target-loss, and motion validation.

**Release review fixes:** stale-request latching, invalid timestamps, and queued
pre-fault target selection now have regression tests. CI uses the shared
`m20_functional` label, including both offline scenarios. Passing software tests
is not approval to run live control without physical commissioning.

M20 Pro follow-layer adaptation for ROS 2 Foxy on GOS. This repository is not
the M20 navigation stack: NOS factory `drmap` remains responsible for mapping,
localization, planning, charging integration, and maps. AOS remains responsible
for locomotion. `jie_deamon` consumes a fused lidar cloud, projects a 2D scan,
computes a candidate follow velocity, and applies a fail-closed gate before the
optional AOS UDP bridge.

The earlier September 11 deployment at `/home/user/m20_fixed_20260911` predates
the new selection-service API. Publishing to GitHub does not update that robot
installation. The examples below use a **fresh** `/home/user/m20_release_ws`;
clone the current source and rebuild rather than merely sourcing an old install.
The older workspaces remain comparison snapshots. Verify the source commit and
`ros2 pkg prefix jie_deamon` in every terminal. Do not set
`commissioned:=true` or start the live bridge until the acceptance gates in
[SOP 8](#8-sop-controlled-live-walk-test) are complete.

## Current Follow Policy

- Select a point cluster manually; human classification is not required.
  Front-lidar range 1.5 m is approximately body-frame `(1.82028, 0)` under the
  existing extrinsic assumption, not a new calibration or a permanent target lock.
- The desired planar distance from the body origin to the selected cluster is
  1.2 m. Distance and bearing hysteresis select holding, turning, or approaching.
- Nonzero forward intent is 0.22-0.30 m/s; nonzero yaw intent has magnitude
  0.52-0.60 rad/s. The M20 profile turns before advancing and commands no lateral
  motion or automatic reverse. These are software limits, not measured robot speeds.
- The profile requires basic gait `4097` (`0x1001`). Do not switch gait without
  revalidating the controller and guard contract.
- Association requires at least three scan points. Ambiguous candidates, target
  loss, or invalid/stale scans latch a stop; returning points do not automatically
  resume following. Reselect, verify, enable calculation, then separately arm.
- M20 selection uses `/robot_nexus/select_target`: a single atomic request carries
  coordinates, frame, issue time, and the server's current fault epoch. Use
  `ros2 run jie_deamon m20_select_target.py 1.82028 0`; this command never enables
  or arms. The old unstamped `/robot_nexus/target` Point topic is legacy-only.
- Body-clearance APF can limit the speed budget. If it cannot support a valid
  gait speed, the controller stops; the full front/rear stop zone is unchanged.
- Tracking and scan freshness use an acquisition-to-use budget of 0.5 s, with
  already elapsed message age deducted. Command timeout remains 0.3 s.
- Runtime configuration and clock-mode changes are rejected. Edit configuration
  while stopped and restart; use the target topic and services for operator actions.

## Safety Boundary

- Never run `drmap mapping`, factory navigation, charging, or another velocity
  controller together with this bridge.
- `dry_run:=true` never opens the AOS UDP control socket. Guarded velocity is
  a preview only.
- `commissioned:=true` is an operator attestation, not an automated safety
  check.
- A zero ROS velocity is a stop request, not a hardware emergency stop. Keep
  the factory remote and an emergency-stop operator present for every live test.
- Do not commit passwords, bags, maps, PCD files, runtime logs, or generated
  `build/`, `install/`, and `log/` directories.

## Architecture

```text
front/rear lidar packets
  -> NOS multicast relay (raw UDP only)
  -> GOS factory decoder and fusion
  -> /LIDAR/POINTS (merged PointCloud2, body coordinates, lidar_link label)
  -> pointcloud_to_laserscan -> /m20/scan
  -> robot_nexus -> /m20/cmd_vel_raw
  -> m20_bridge gate -> /m20/cmd_vel_guarded
  -> optional AOS basic_server UDP Type=2, Command=25
```

The factory configuration uses `send_separately: false`. NOS and GOS have the
same factory lidar configuration hash. The relay forwards raw packets; GOS
applies its own factory decoder/fusion configuration. Do not switch to separate
mode or apply the front/rear extrinsics again. `target_frame: ''` deliberately
preserves the fused physical coordinates despite the `lidar_link` label.

| Host | Address | Responsibility |
| --- | --- | --- |
| AOS | `10.21.31.103` | Factory locomotion and `basic_server` |
| NOS | `10.21.31.106` | Factory lidar path, SLAM, planning, charging, PTP master |
| GOS | `10.21.31.104` | This package and factory lidar decode/fusion |

## ROS Interfaces

| Direction | Name | Meaning |
| --- | --- | --- |
| Subscribe | `/LIDAR/POINTS` | Factory fused `sensor_msgs/msg/PointCloud2` |
| Publish | `/m20/scan` | 2D scan used by tracker and bridge |
| Publish | `/m20/cmd_vel_raw` | Tracker intent, never proof of movement |
| Publish | `/m20/cmd_vel_guarded` | Gate preview in dry-run; AOS command in live mode |
| Publish | `/m20/bridge_status` | JSON gate state and current reason |
| Publish | `/robot_nexus/tracking_status` | Target validity, selection generation, controller and APF diagnostics |
| Service | `/robot_nexus/select_target` | Timestamped, epoch-checked selection using `rcl_interfaces/srv/SetParametersAtomically` |
| Legacy only | `/robot_nexus/target` | Unstamped Point input; absent in the gait-aware M20 profile |
| Service | `/robot_nexus/enable_follow` | Timestamped enable bound to selection ID and fault epoch; no platform arm |
| Service | `/robot_nexus/set_moving` | `false` always disables; `true` rejected in the M20 profile |
| Service | `/m20/arm` | Manually arms/disarms a healthy bridge |

## 1. Build and Offline Tests

Use Foxy on the M20 host. Humble compatibility checks do not replace Foxy/M20
validation.

```bash
mkdir -p /home/user/m20_release_ws/src
git clone https://github.com/bernardleex526/deamon2.git /home/user/m20_release_ws/src/jie_deamon
cd /home/user/m20_release_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select jie_deamon --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select jie_deamon --return-code-on-test-failure \
  --ctest-args -L m20_functional --output-on-failure
colcon test-result --verbose
cd src/jie_deamon
python3 -m unittest discover -s test -v
```

The Python suite includes protocol, guard, scan, and loopback-UDP tests. It
does not contact AOS. Focused C++ tracker tests and ROS smoke tests are separate
from inherited repository-wide lint failures.

For an isolated synthetic test, use only a non-robot domain:

```bash
source /opt/ros/foxy/setup.bash
source /home/user/m20_release_ws/install/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch jie_deamon m20.launch.py cloud_topic:=/m20/mock_points
```

In another terminal with the same isolated environment:

```bash
ros2 run jie_deamon m20_mock_inputs
```

Never start `m20_mock_inputs` in robot Domain 0.

## 2. SOP: Connect and Inspect Existing State

From the engineering computer, inspect before starting anything. The SSH aliases
`104` and `106` resolve to GOS and NOS. Authenticate interactively; do not put
credentials in a script or a repository file.

```bash
ssh 106
systemctl show multicast-relay.service -p ActiveState -p SubState -p UnitFileState

ssh 104
ps -eo pid,user,args | grep -E \
  '[r]os2 launch jie_deamon|[m]20_bridge|[r]obot_nexus|[p]ointcloud_to_laserscan'
```

1. If the relay is inactive, start it for the current boot only on the host
   that owns the unit:

   ```bash
   sudo systemctl start multicast-relay.service
   ```

2. If all four GOS follow processes already exist, do not start another launch.
   Continue at [SOP 5](#5-sop-start-and-verify-one-dry-run-instance).
3. If planner, localization, charging, mapping, or another velocity source is
   active, this is observation-only. Do not prepare a live follow test.

## 3. SOP: Prepare GOS ROS Environment

Use the same GOS root environment that receives the factory DDS shared-memory
publisher path. Source before enabling `set -u` or starting diagnostics.

```bash
sudo -i
export ROS_DISTRO=foxy
source /opt/robot/scripts/setup_ros2.sh
source /home/user/m20_release_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 pkg prefix jie_deamon
```

Do not create a process that links vendor DrDDS and Foxy Fast DDS together. This
package uses the existing factory publisher/standard ROS boundary.

## 4. SOP: Prove Fresh Lidar Before Follow

Run this before every new launch. A driver log saying `send success` is not
application sensor acceptance.

```bash
ros2 run jie_deamon m20_cloud_probe.py --seconds 12 --cloud-only
```

Pass only when `cloud.count` continuously rises, point count/data bytes are
nonzero, fields include `x`, `y`, `z`, message age is fresh, and
`usable_fresh_samples` is `true`. The observed M20 cadence is about 10 Hz.

If the check fails, stop here and inspect without changing factory lidar setup:

```bash
systemctl status rsdriver.service --no-pager
journalctl -u rsdriver.service -n 40 --no-pager
```

Do not start `drmap mapping` solely to make a follow test progress.

## 5. SOP: Start and Verify One Dry-Run Instance

Only when the cloud probe passes and the process inspection found no instance:

```bash
ros2 launch jie_deamon m20.launch.py \
  dry_run:=true commissioned:=false enable_web:=false
```

Keep that terminal open. In another GOS root terminal with the environment from
[SOP 3](#3-sop-prepare-gos-ros-environment):

```bash
ros2 run jie_deamon m20_cloud_probe.py --seconds 12 --scan-topic /m20/scan
ros2 param get /m20_bridge dry_run
ros2 param get /m20_bridge commissioned
timeout 5s ros2 topic echo /m20/bridge_status std_msgs/msg/String
timeout 5s ros2 topic echo /robot_nexus/tracking_status std_msgs/msg/String
```

These bounded topic readers do not call arm, set-moving, or publish control;
`timeout` normally exits with code 124 when the observation window ends.
For a clear static area, require fresh scans, frame `lidar_link`, no malformed
scan, `armed=false`, and zero guarded velocity. Inspect the full scan to confirm
front, rear, and side clearance; zero guarded velocity alone is not proof of it. A bridge
reason can remain `invalid scan or obstacle` after later clear scans because the
gate latches until a manual arm. Do not arm merely to clear the displayed reason.

Before selection, `tracking_valid=false` and a target-unavailable/unselected
reason are expected. Use the cloud/scan probe and tracking/bridge status together;
bridge status alone is not a complete obstacle-clearance report.

## 6. SOP: Remote RViz on the Engineering Computer

Use X11 forwarding only for visualization, never as a safety display. Root can
be required to see the same DDS shared-memory path as the factory publisher.

```bash
ssh -X 104
sudo -i
export ROS_DISTRO=foxy
source /opt/robot/scripts/setup_ros2.sh
source /home/user/m20_release_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
rviz2
```

Configure RViz manually:

| Item | Value |
| --- | --- |
| Fixed Frame | `lidar_link` |
| PointCloud2 topic | `/LIDAR/POINTS`, Best Effort |
| LaserScan topic | `/m20/scan`, Best Effort |
| Cloud color | AxisColor, Z axis |

Confirm visible points in the actual RViz window or a screenshot. An RViz
process or ROS graph entry alone is not rendering proof. The September 8 test
used software OpenGL at about 1 FPS, suitable only for inspection.

## 7. SOP: Static Target and Dry-Run Preview

Prerequisites: clear stop zone, stationary robot, emergency-stop operator, and
an identified operator-selected point cluster in scan/RViz. This does not arm the bridge or
open an AOS socket.

1. Record the target's actual body-frame XY coordinate from scan data.
2. Send one target and inspect tracking status before enabling calculation:

   ```bash
   ros2 run jie_deamon m20_select_target.py 1.82028 0 --frame lidar_link
   timeout 5s ros2 topic echo /robot_nexus/tracking_status std_msgs/msg/String
   ```

3. Confirm the selected cluster is correct and `tracking_valid=true` remains
   stable, then enable the calculation preview:

   ```bash
   ros2 run jie_deamon m20_enable_follow.py
   timeout 5s ros2 topic echo /m20/cmd_vel_raw geometry_msgs/msg/Twist
   timeout 5s ros2 topic echo /m20/bridge_status std_msgs/msg/String
   ```

   Require `dry_run=true`, `armed=false`, and zero guarded velocity in this
   preview. Nonzero raw velocity proves only algorithm intent, not robot motion.
4. Immediately disable calculation after the bounded observation:

   ```bash
   ros2 service call /robot_nexus/set_moving std_srvs/srv/SetBool '{data: false}'
   timeout 5s ros2 topic echo /m20/bridge_status std_msgs/msg/String
   ```

The old September 8 lateral output is historical evidence, not the current
M20 control policy. The current policy commands Y=0. On any lost, ambiguous,
or stale target, inspect the cause before reselecting; do not automate repeated
reselection or arming. Physical occlusion and stop acceptance are still required.
The selection command reads a fresh `fault_epoch` from tracking status and makes
one request. If the epoch changes while it is queued, the request is rejected;
read the reason and inspect the scene before issuing a new manual selection.
The enable tool also sends a single timestamped request bound to the observed
selection ID and fault epoch. Faults, reselection, and explicit disable invalidate
queued enable requests. It does not call `/m20/arm`; a live bridge may execute
intent only when separately armed. Never use these tools as an automatic retry loop.

## 8. SOP: Controlled Live Walk Test

This is an acceptance checklist, not a motion command. Do not proceed without
a responsible operator explicitly authorizing the test.

1. Prepare a bounded clear test area, factory remote, hard emergency stop, and
   a second observer.
2. Confirm NOS mapping, navigation, planner, charging, and any other velocity
   owner are idle. Record how normal factory operation will be restored. Do not
   stop `basic_server`, `rl_deploy`, radar, or time synchronization services.
3. Use the factory interface to verify fresh BasicStatus: `MotionState=17`,
   `ControlUsageMode=1`, `Direction=0`, `Charge=0`, `HES=0`, `Sleep=0`, and a
   `Gait=4097`. Read telemetry without mode or velocity commands:

   ```bash
   ros2 run jie_deamon m20_status_probe.py --seconds 6
   ```

4. Verify actual velocity units/signs, gait minimums, and emergency-stop
   behavior. Bridge limits are m/s and rad/s; no physical response has yet been
   accepted on this robot.
5. Stop the one dry-run launch and confirm its processes are gone.
6. Only after steps 1-4 pass, start live:

   ```bash
   ros2 launch jie_deamon m20.launch.py \
     dry_run:=false commissioned:=true enable_web:=false
   ```

   Live startup sends heartbeat and zero-velocity UDP packets. It does not take
   ownership from factory tasks, change posture, select gait, or arm itself.
7. Inspect stable `/m20/bridge_status` and complete BasicStatus while follow is
   disabled. An ACK is not movement acceptance.
8. Reselect the target after restarting: selection is not persisted across
   launches. Verify fresh tracking, use `m20_enable_follow.py`, then `/m20/arm` only for an
   explicitly authorized, time-bounded trial with a responsible operator.
   Validate forward and yaw response; this profile does not command Y motion.
   Do not increase limits or suppress obstacles simply to clear a gate.
9. Repeat acceptance for target loss, occlusion, near obstacle, scan stop,
   bridge stop, network loss, manual takeover, charging state, and emergency
   stop. Measure real stop distance and robot-side timeout behavior.

Only after every case passes may this repository claim supervised live-follow
acceptance. Unattended autonomous following is out of scope.

## 9. SOP: Stop and Restore

For dry-run, disable follow calculation and use `Ctrl+C` in the launch terminal:

```bash
ros2 service call /robot_nexus/set_moving std_srvs/srv/SetBool '{data: false}'
ps -eo pid,user,args | grep -E \
  '[r]os2 launch jie_deamon|[m]20_bridge|[r]obot_nexus|[p]ointcloud_to_laserscan'
```

For live operation, disarm, verify physical standstill, stop the bridge, then
restore the recorded factory planner/control state. After reboot, recheck the
relay because it can be active for the boot while disabled at boot.

## Evidence and Documents

- [September 11 controller fixes and current SOP](docs/M20_FIXES_20260911.md)
  describes the current profile, generation/freshness gates and test boundaries.
- Before the selection-service update, local Humble and native GOS/Foxy runs
  passed six functional groups and native replay of a 233-frame scan sequence
  had no target loss and commanded Y=0. Those results describe that snapshot.
- The current functional label adds node callback-order, malformed timestamp,
  queued selection and recovery coverage; CI runs the same label for Foxy and
  Humble. Local runs and replay are not a GitHub Actions success claim or physical
  following acceptance. In-memory armed/guard output is not a live AOS command.
- The release-review revision was tested locally on Humble: seven functional
  groups, including 19 tracker/controller tests, eight node ordering tests,
  37 Python unit tests and five bridge/request-contract tests. This revision
  has not been redeployed to GOS; its Foxy CI result must be checked separately.
- Full repository lint is not all green; inherited style/copyright issues and
  third-party/schema tooling limitations are tracked separately from functional
  tests. Do not describe the focused test selection as full-suite acceptance.

- [Offline follow simulation and September 9 algorithm fixes](docs/M20_OFFLINE_FOLLOW.md)
  runs the production C++ tracker and Python guard without ROS nodes or robot
  connections. This document describes the historical minimal adaptation;
  use the September 11 document for the current gait-aware profile. Real robot
  commissioning is still required.
- [M20 adaptation boundary and protocol reasoning](docs/M20_ADAPTATION.md)
- [Expanded runbook and acceptance template](docs/M20_RUNBOOK.md)
- [September 7 GOS deployment record](docs/M20_DEPLOYMENT_2026-09-07.md)
- [September 8 near-obstacle observation](docs/M20_GUARD_CHECK_2026-09-08.md)
- [September 8 sensor, RViz, and dry-run record](docs/M20_DRY_RUN_2026-09-08.md)
- [Vendor evidence index](docs/M20_SOURCES.md)

## Repository Layout

```text
config/m20.yaml                 M20 tracker, projection, and bridge parameters
launch/m20.launch.py            Follow algorithm plus dry-run/live bridge
launch/m20_mapping.launch.py    Passive observer for NOS mapping
m20_adapter/                    UDP framing, gate, probes, mock source
scripts/m20_cloud_probe.py      Bounded read-only sensor probe
scripts/m20_status_probe.py     Heartbeat-only AOS telemetry probe
scripts/m20_select_target.py    One-shot stamped selection; no enable or arm
scripts/m20_enable_follow.py    One-shot epoch-bound calculation enable; no arm
test/                           Isolated protocol, guard, tracker, ROS tests
docs/                           Evidence, deployment, and acceptance records
```

## License

MIT. Original upstream D1 components remain in the tree but are not supported
as M20 Web, Android, action, or direct-control interfaces. M20 launches disable
those paths explicitly.
