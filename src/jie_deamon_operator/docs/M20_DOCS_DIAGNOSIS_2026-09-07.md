# M20 lidar documentation cross-check, 2026-09-07

## Sources read in this run

The current published pages were downloaded with curl and their
`ssr_frontend_data.wordSSRContent.html` was parsed using JSON and an HTML parser.
The web tool did not return readable content; this run did not rely solely on
the older repository snapshots.

- [Topic reference](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/oP0MALyR8k73gzN3iYagqwbK83bzYmDO), sections 1-3.
  Extract: `/tmp/deamon2_vendor_topics_current.txt`, lines 23-82.
- [Lidar guide](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/20eMKjyp81RbA0YbUe21nQMZWxAZB1Gv), sections 4.2, 5 and 6.
  Extract: `/tmp/deamon2_vendor_lidar_current.txt`, lines 105-243.
- Linked [DDS overview](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/o14dA3GK8g5y4QGySnyegPppV9ekBD76), sections 1 and 2.
  Extract: `/tmp/deamon2_vendor_dds_current.txt`, lines 20-58.

## Documented contract versus current evidence

| Contract | Evidence and conclusion |
| --- | --- |
| `/LIDAR/POINTS` is PointCloud2, nominal 10 Hz, merged by default | Native discovery confirms the topic/type; application frequency is NOT verified. |
| GOS additionally needs raw-lidar multicast forwarding | NOS relay and GOS rsdriver are active; GOS logs show both lidars with error_code=0 and positive cld_size. Raw input is present, but that is not DDS sample-delivery proof. |
| GOS already decodes raw lidar; only external computers need to implement an SDK receiver | Keep deamon2 on GOS; a NOS ROS-topic whitelist change is not the documented primary path. |
| `send_separately: false` is the default; separate mode affects navigation/localization | Current GOS file is false. Do not change to separate mode to diagnose zero samples. Merely discovering POINTS2 is not evidence it is actively transmitting. |
| Configuration is per host | Editing NOS is not a substitute for checking GOS rsdriver. |
| Relay introduced in V1.1.7; startup and enable-at-boot are distinct | Relay is active but disabled for boot. This is a reboot persistence gap, not an explanation for current loss while it is active. |
| PTP should report 0x1, timestamp difference normally 100-200 ms | Recent sampled logs did not expose those fields. PTP remains unverified; do not change timestamps to hide the gap. |
| Internal ROS CLI requires vendor setup; `su` is recommended | GOS setup script was inspected: it sources Foxy and selects factory XML, inserting the local IP only if absent. GOS already has 127.0.0.1 and 10.21.31.104 in that XML. Root + official ROS-tool receipt has not yet been demonstrated. |

## Correction to previous diagnoses

The operator supplied root UDP-only and root SHM-enabled native probe runs;
both matched two writers and received zero samples. The SHM-enabled view
includes SHM locators, so the earlier UDP-only locator output must not be
interpreted as an exhaustive list of publisher transports.

More importantly, the native probe has not passed its independent positive
control. A separate synthetic native publisher and receiver in domain 83,
UDP loopback only, matched but produced no reader callback even for small
payloads. After correcting a bool-versus-ReturnCode write-counter mistake in
the synthetic publisher, 100 writes reported success. strace on these OWN
synthetic processes showed RTPS DATA sends and ACKNACK exchanges, but no
PointCloud2 deserialize callback. No robot-domain synthetic publisher was used.
The native probe was then changed to poll in addition to callbacks and compiled,
but that revision has NOT been behavior-tested: user-directed documentation
review took priority. Do not use its zero count to assign a vendor root cause.

This establishes an unresolved native test-client/SDK/runtime problem, not a
confirmed firmware defect, lost raw lidar, missing whitelist, or ROS 2 ABI fault.
Existing standard-ROS receipt failures remain observations requiring their own
validated official baseline. Earlier emphasis on permissions/whitelists was
premature. The two pages alone do not identify a unique root cause.

## Next discriminating check

Use the documented ROS 2 path on GOS, not the unvalidated native probe. The
operator authenticates locally; no password is stored or supplied by the agent.

```bash
su
export ROS_DISTRO=foxy
source /opt/robot/scripts/setup_ros2.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout -s INT 15s ros2 topic hz /LIDAR/POINTS
timeout -s INT 10s ros2 topic hz /IMU
exit
```

Timeout is expected at the time limit. Rate output proves samples arrived;
no output is a failed receipt check, not proof that the sensor is off.
POINTS works: fix application environment using that baseline.
IMU works but POINTS does not: focus on GOS lidar publisher/type/transport.
Neither works: first inspect the common ROS/DDS environment and runtime.
Do not restart factory services, change maps, enable motion or upgrade firmware
merely because one of these checks is negative.
