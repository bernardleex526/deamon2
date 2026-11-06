# M20 Pro GOS deployment, 2026-09-07

Latest correction: [documentation-based diagnosis](M20_DOCS_DIAGNOSIS_2026-09-07.md).
The operator's root SHM probe also returned zero. The native probe's positive
control subsequently failed, so its zero count is not sufficient evidence to
assign a firmware, permissions or whitelist root cause. Earlier hypotheses
below are historical, not confirmed diagnoses.

## Follow-up: native DDS endpoint inspection

### Root UDP comparison and SHM correction

The operator ran the GOS UDP-only probe as root and supplied its output:
two writers matched, `TOTAL_CLOUDS=0`. This rules out ordinary-user permissions
as the sole explanation for that UDP-only failure; it does not test root SHM.

The GOS diagnostic now also accepts `read-shm`, adding a 64 MiB SHM transport
while retaining UDP and keeping DDS data sharing off. Ordinary-user testing
matched two writers but received zero samples. Crucially the same writers now
show `SHM:[_]:7411` and `SHM:[_]:7413` as well as UDP loopback. The earlier
UDP-only listener exposed only its supported locator view: saying the vendor
offered ONLY UDP loopback was too strong. The table below records the earlier
UDP view, not an exhaustive list of publisher transports.

Next operator comparison on GOS (no writes to vendor services/configuration):

```bash
su -c '/home/user/m20_ws/deamon2_dds_inventory 127.0.0.1 read-shm'
```

The output must start `MODE=read-shm SHM=1`. It exits after about 12 seconds.
Root SHM sample delivery remains unverified until that result is available.

Read-only native Fast DDS 2.14 discovery on GOS confirmed domain 0 writers:

| Topic | Type | Advertised unicast locator |
| --- | --- | --- |
| `rt/LIDAR/POINTS` | `sensor_msgs::msg::dds_::PointCloud2_` | `127.0.0.1:7411` |
| `rt/LIDAR/POINTS2` | same | `127.0.0.1:7411` |
| `rt/LIDAR/POINTS` (second writer) | same | `127.0.0.1:7413` |

NOS loopback inspection separately found `rt/LIDAR/POINTS` at
`127.0.0.1:7455` and `127.0.0.1:7459`, and `rt/LIDAR/POINTS2` at
`127.0.0.1:7455`; its loopback subscriber matched two and received zero samples.

These writers advertise RELIABLE and VOLATILE. Native discovery sees them even
when the ordinary ROS graph query lists no publishers. Topic spelling is now
verified, not inferred from example/configuration text. Writer/process identity
and whether the second writer supplies usable samples remain unverified.

A native subscriber using installed vendor PointCloud2 types, RELIABLE QoS,
UDP-only loopback and data sharing disabled matched both writers but received
zero samples over 12 seconds. The existing DrDDS receiver with `--network lo
--shm` also received zero samples. A native NOS subscriber using loopback plus
its 33-network interface likewise received zero samples. No root comparison
has run: noninteractive sudo requires authentication, and root SSH key access
is unavailable. No password was placed in commands or files.

The active NOS IMU and other vendor endpoints already advertise both
`10.21.33.106` and `10.21.31.106`, although `/opt/robot/fastdds.xml` only lists
loopback and the 33 address. Thus that XML is not evidence of every vendor
process's effective transport. Do not assume changing it will change the
lidar writer. If a local-interface whitelist eventually needs adjustment on
NOS, its 31-network address is `.106`, not GOS's remote `.104`.

Conclusion: missing `.104` in NOS XML is not an established root cause. Local
sample delivery fails too; privileged inspection of writer environment,
actual loaded transport, SHM access and bounded RTPS traffic remains necessary.
No factory XML, service or motion state was changed in this investigation.

Diagnostic source: local `/tmp/deamon2_dds_inventory.cpp`; GOS source/binary
`/home/user/m20_ws/deamon2_dds_inventory.cpp` and `deamon2_dds_inventory`;
NOS source/binary `/home/user/deamon2_dds_inventory.cpp` and
`/home/user/deamon2_dds_inventory`. These are bounded diagnostic tools, not
installed services or controllers. The second argument `read` adds a
PointCloud2 subscription only; it never creates an application DataWriter.

## Decision and source

Deploy `jie_deamon` on GOS `10.21.31.104`, in
`/home/user/m20_ws/src/jie_deamon`; installed output is
`/home/user/m20_ws/install/jie_deamon`. Local source is `/home/lee/deamon2`.
GitHub main and local HEAD were both
`82b8b990090c091a1771a13ad4dd0472c73c50ce` at inspection. Existing local edits
were preserved; GOS initially had the same files without `.git`. Adapted
source was synchronized with rsync, excluding `.git` and `__pycache__`,
without deletion. This is an uncommitted working-tree deployment, not a release.

NOS `10.21.31.106` retains factory SLAM/localization/planning. Its actual
`/usr/local/bin/drmap` points to
`/opt/robot/share/slam/scripts/drmap_scripts/drmap`; `mapping` invokes the
vendor `mapping.sh`. No `drmap mapping` or `stop_mapping` was executed here.
AOS `10.21.31.103` retains motion control, with no software installed there.
deamon2 is a tracker/control application, not a replacement SLAM implementation.

Documentation evidence: [existing vendor source index](M20_SOURCES.md),
local vendor chapters `07_计算平台与资源分配.html`, `17_激光雷达.html`,
and `30_建图与地图管理.html` in `/home/lee/m20pro/m20文档/doc/`.
The supplied DingTalk page did not return readable text through the web tool
in this run; local vendor copies, repository snapshots, and live host files
were used instead. Architecture below distinguishes observation from inference.

## Verified host and sensor state

- All three hosts were reachable by key-based SSH through AOS. GOS and NOS
  identify themselves as `CA9B_GOS` and `CA9B_NOS`; their version files contain
  `104_v1.0_v2.0_0728` and `106_v1.0_v2.0_0728`. These strings alone do not
  establish a release-note version such as V1.1.8.
- GOS has ARM64/Foxy and approximately 9 GiB available memory at inspection.
  NOS was already running localization, planner and sensor processes with
  load average about 8.9; GOS load average was about 0.7.
- NOS `multicast-relay.service` was active but disabled for boot. Its existing
  `/usr/bin/multicast.py` receives via `10.21.33.106`, forwards via
  `10.21.31.106`, and handles front/rear groups `224.10.10.201/.202` on
  MSOP ports `6691/6692` and DIFOP ports `7781/7782`. We did not change it.
- GOS `/opt/robot/share/node_driver/config/config.yaml` has
  `send_separately: false`, both RSAIRY drivers, and `dds_frame_id: lidar_link`.
  Preserve this factory fusion; do not switch to separate mode or apply a
  second body transform. Physical point coordinates are documented, not
  independently verified from received application data in this run.
- GOS `rsdriver.service` runs as root. At 14:12 CST its journal showed both
  lidar status=1/error_code=0, roughly 51k/49k points, and merged sends of
  roughly 100k points. The journal also included a smaller 47k-point frame;
  this is not proof of continuously healthy dual-lidar coverage.
- The GOS config text names `/lidar_points` while documentation names
  `/LIDAR/POINTS` and installed samples name `/LIDAR_POINTS`. Do not rewrite
  system configuration based on those names alone; inspect actual DDS endpoints.

Selected path: front/rear raw lidar -> NOS multicast relay -> GOS vendor
decoder/fusion -> application PointCloud2 -> LaserScan -> deamon2.
The raw relay/decoder stages have live evidence; application delivery does not.

## Changes in this run

1. Fixed the passive mapping launch node name to match `m20_cloud_to_scan`
   in `config/m20.yaml`. Previously its different name silently skipped the
   height/range configuration.
2. Changed the synthetic smoke domain from 233 to 83. This machine's Fast DDS
   rejected domain 233 with `Calculated port number is too high`.
   Synthetic tests remain localhost-only and dry-run.
3. Added `UdpClient.heartbeat()` and stopped active `1002/6` polling in the
   bridge. Current heartbeat-only reception was measured at 2 Hz BasicStatus.
   No live velocity bridge was started to verify motion output.
4. Added and installed bounded sensor and heartbeat-only status probes, and a
   passive launch smoke test checking effective parameters, synthetic geometry
   and absence of velocity publishers. Probes reject incomplete/stale telemetry
   and empty/stale sensor samples; neither constitutes motion acceptance.

## Verification

- GOS ARM64/Foxy `colcon build --packages-select jie_deamon`: passed.
- Python unittest: 28 tests passed, including loopback UDP and heartbeat-only
  registration. Loopback tests do not send velocity to the robot.
- C++ `test_m20_tracker`: 3 tests, 0 failures, 0 errors.
- `test/ros_m20_smoke.py`: passed default zero, forward tracking, target loss,
  obstacle latch and input-loss cases using synthetic inputs, domain 83,
  localhost-only, dry-run. Nonzero previews existed only in that isolated test.
- `test/ros_m20_mapping_smoke.py`: passed scan reception, effective parameters
  `[-0.1, 0.5, 12.0]`, no `robot_nexus`/`m20_bridge`, and no velocity publishers.
- SIGINT cleanup can print exit code -2 for child processes; behavior assertions
  passed, but graceful signal handling has not been comprehensively repaired.
- Full ament lint is NOT green. Existing copyright/style failures and vendored
  `httplib.h` tool-language errors were already present in pre-run results;
  no broad formatting or license rewrite was attempted.
  The full-run snapshot before final probe hardening reported 949 result entries,
  1 error and 921 failures (including aggregated CTest entries). This is not
  a count of independent behavior tests. Final scoped behavior tests were rerun.
- AOS heartbeat-only probe: six heartbeats, twelve BasicStatus messages in
  six seconds; MotionState=17, Gait=4097, ControlUsageMode=0, Direction=0,
  HES=0, Charge=0, Sleep=0. The guard correctly requires navigation mode=1,
  so this state does not authorize movement. No mode was changed.
- Final installed probes repeated the same outcome: `complete_fresh_status=true`
  for AOS, `cloud.count=0` and `usable_fresh_samples=false` for lidar.

The installed diagnostics can be invoked after sourcing the workspace:

```bash
ros2 run jie_deamon m20_cloud_probe.py --seconds 12 --cloud-only
ros2 run jie_deamon m20_status_probe.py --seconds 6
```

The status probe sends heartbeats, but never velocity or mode commands.

## Remaining blocker

Ordinary-user standard ROS probes received zero PointCloud2 messages and no
ROS publisher endpoints over 10-15 seconds, with the factory XML, default
transport, and an existing UDP-only profile. A bounded existing DrDDS
sensor-only receiver matched two publishers but reported
`updated=no received=0 forwarded=0`. The receiver was stopped after the test;
no alternate SLAM stack was started.

Root-owned shared-memory segments and root-owned vendor publishers make
permissions/transport compatibility a hypothesis, NOT an established cause.
`sudo -n` was denied because authentication is required. Further privileged
diagnostics require the operator at a GOS terminal; do not share a password.
One next read-only comparison is:

```bash
cd /home/user/m20_ws/src/jie_deamon
sudo bash --noprofile --norc -c '
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
python3 /home/user/m20_ws/src/jie_deamon/scripts/m20_cloud_probe.py --seconds 12 --cloud-only
'
```

This subscribes only. If root receives data, compare SHM permissions and DDS
configuration before choosing a narrow read-only gateway. Do not run the
whole controller as root. If root also receives nothing, inspect publisher
environment, actual topic/domain/type, locators and live RTPS traffic before
changing any service. Do not chmod/unlink shared memory or restart factory
services blindly.

## Operating boundary

After sensor delivery is resolved, use `m20_mapping.launch.py` only for
passive observation alongside NOS mapping. Source `/opt/ros/foxy/setup.bash`
and `/home/user/m20_ws/install/setup.bash` explicitly; the vendor
`setup_ros2.sh` can edit `/opt/robot/fastdds.xml`, so it was not sourced here.

Do not enable live following until real frames, ages, body coordinates,
coverage, single control ownership, mode/gait and stopping behavior have
been validated. No raw or guarded velocity was published in robot domain 0;
no AOS speed, pose, mode, gait or emergency-stop command was sent. No factory
service, map, driver configuration, boot setting or system ROS was changed.
The current delivery is source/build/offline adaptation plus telemetry
validation, NOT live sensor or physical-follow acceptance.
