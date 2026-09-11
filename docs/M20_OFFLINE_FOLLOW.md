# M20 Minimal Follow Adaptation and Replay SOP

Updated: 2026-09-09. Local changes only; GOS deployment is not implied.

## Scope

The experimental target-directed lateral policy and its parameter have been
removed at the operator's request. Preserve upstream target association,
centroid tracking, optional Kalman filtering, forward-distance response, yaw
response, and APF. Do not add another policy based on ideal synthetic results.

Remaining changes:

- Configurable scan orientation, following distance, corridor/body dimensions;
  M20 topic remaps, sensor QoS, and the existing protocol/guard adapter.
- Invalid range filtering, no matched target => zero command, use the updated
  centroid, and empty scan => clear cached velocity and publish zero.
- Reproduced corridor fix: initialize bounds with the correct signs, select
  nearest boundaries and use their midpoint. One right obstacle previously
  caused about +0.600 m/s correction; the regression now expects +0.100 m/s.
  This does not prove all field oscillations fixed.
- Read-only per-frame diagnostics and offline tests.

Forward speed uses `target_x - follow_distance`; yaw uses target bearing.
No target-directed lateral translation is added. Existing corridor/APF terms
can still produce Y velocity. This is NOT a strict "turn in place first"
state machine: upstream can turn and advance simultaneously, and can request
reverse when the forward distance is below the setpoint. Expose these behaviors
in real-bag comparison rather than adding an untested policy.

## Offline Smoke Test

The runner calls the production C++ tracker and Python `inspect_scan`/`Guard`.
It opens no robot socket, creates no ROS node, and calls no hardware services.
Its mock status and arm are in-memory test objects only.

```bash
source /opt/ros/humble/setup.bash
cmake -S /home/lee/deamon2 -B /tmp/deamon2-follow-build \
  -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/deamon2-follow-build \
  --target test_m20_tracker m20_follow_scenario robot_nexus -j2
/tmp/deamon2-follow-build/test_m20_tracker
ctest --test-dir /tmp/deamon2-follow-build -R '^m20_follow_offline_' \
  --output-on-failure
/usr/bin/python3 /home/lee/deamon2/scripts/m20_follow_offline.py \
  --tracker /tmp/deamon2-follow-build/m20_follow_scenario \
  --follow-distance 1.2 --output /tmp/m20-minimal-follow-results
```

The synthetic scene uses a fixed robot origin, static room and circular target,
seeded once. The 80-second input moves from 2 m to 3 m, then to 2 o'clock,
center, 10 o'clock, center, target removal, near obstacle, and empty scan.
The 2 m and 1.2 m setpoint tests are parameter regressions, not two policies.
Production YAML remains at 1.2 m. There is no robot pose integration or physics.

Outputs: `tracker.csv`, `guarded.csv`, `summary.json`, optional `velocity.png`.
Old plots of the removed lateral policy are not current results.
The simulated gait 12290 is not a real gait selection. Existing gait minima
and per-axis clamps are unchanged; gait 4097 cannot pass Y velocity under the
current 0.30 m/s cap. No protection limit is relaxed.

## Same Real Bag Comparison

Compare upstream baseline against minimal adaptation, NOT a lateral experiment.
Upstream snapshot inspected: `9403185c0cf99a0b415ddd4f3b4215b44dd0809c`.

Use identical real scan samples, timestamps, initial target, geometry, setpoint,
and Kalman setting. Adapt upstream scan orientation equivalently before testing;
otherwise its built-in 180-degree flip invalidates the comparison. Run both
through the same isolated guard with the same synthetic status and clock.

Record target centroid/count, corridor term, APF term, raw X/Y/yaw, gate output
and reason. Compare centered-target bias, left/right yaw signs, forward/reverse
requests at large bearings, association jumps, target loss, near obstacles,
empty scans and explicit no-message timeouts. Do not republish recorded control
topics or play bags into a live robot domain.

The current synthetic runner does not load rosbag. Real-bag comparison is
pending an available recording and an offline bag reader using the same tracker
entrypoint. Do not claim bag acceptance from synthetic tests.
After comparison: GOS Foxy build, live dry-run, then supervised motion acceptance
with verified control ownership, gait and emergency stop.
