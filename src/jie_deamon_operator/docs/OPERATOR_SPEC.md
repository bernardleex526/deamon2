# Accepted operator interface scope

Keep the deamon2 two-dimensional D1 tracker and M20 UDP Cmd 25 bridge. Add one
GOS command to prepare the environment, check/start the existing NOS relay,
verify fresh cloud, prevent duplicate managed instances, and launch. Do not
automatically switch factory modes, stop planners, or arm after process restart.

Default to a phone HTTP interface with scan target selection, hold-to-move
controls, start-follow, stop, mode, ownership and rejection reasons. Start
combines operator ownership and arm in live mode. Preserve robot status, speed,
scan, command and operator connection gates. One source may own the controller;
an authenticated other source may stop but may not take over or refresh its lease.

Android uses the same authenticated, sequenced request schema through UDP,
including explicit start/stop and heartbeats. The old unmodified app is not
protocol compatible; no unauthenticated legacy path is enabled for M20.

Do not guess D1-to-M20 posture mappings. Reject unsupported actions explicitly
and explain use of the factory controller. Existing action logic under legacy/
uses a different control mode and does not constitute verified Cmd 25 mappings.

Dry-run never opens an AOS socket or arms the guard. It permits scan-validated
raw velocity preview without inventing BasicStatus. Live remains conditional on
physical acceptance, including resolution of the recorded lateral-follow issue.

Deliver on a new branch of bernardleex526/deamon, preserving its prior package
under legacy/ with COLCON_IGNORE. Do not claim robot or ROS build acceptance
from Windows Python tests alone.
