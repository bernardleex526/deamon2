# M20 GOS read-only guard check, 2026-09-08

Checked the existing live-sensor dry-run launch on GOS, without restarting it.
The standalone diagnostic read `/m20_bridge/get_parameters` and subscribed
to `/m20/scan`, `/m20/bridge_status`, `/m20/cmd_vel_raw`, and
`/m20/cmd_vel_guarded`. It did not call arm/set_moving or publish commands.
Authentication was interactive; no credentials are included in these files.

## Actual runtime parameters

- dry_run: true; commissioned: false
- scan_frame: lidar_link
- stop_front: 0.7 m; stop_back: 0.7 m; stop_half_width: 0.4 m

## Fifteen-second observation

| Metric | Result |
| --- | --- |
| Scan samples | 150 |
| Scan frequency | 10.003 Hz |
| Maximum observed interarrival gap | 0.129 s |
| Source age range | 128.34-259.17 ms |
| Invalid source age | 0 frames |
| Wrong frame label | 0 frames |
| Invalid scans under installed inspect_scan | 0 frames |
| Valid scans with stop-region returns | 81 frames |
| Stop-region returns per frame | 0-16 |
| Guarded velocity messages | 300, all zero |
| Raw algorithm velocity messages | 150, all zero |
| Bridge status messages | 301, all report invalid scan or obstacle |

Closest observed stop-region return: distance 0.44784 m,
x=-0.36685 m, y=+0.25687 m. In the documented body coordinate convention,
this is rear-left. Its physical source has not been identified: a real nearby
object and body/leg returns remain possibilities. No footprint mask or stop
threshold was changed, and no physical-motion or camera verification occurred.

The last scan was valid and clear, with zero stop-region returns. The bridge
remained disarmed and retained `invalid scan or obstacle`. This agrees with
`Guard.update_scan`: a bad scan/obstacle disarms; subsequent clear scans do
not automatically arm or clear the retained reason. The diagnostic reused
the installed `inspect_scan` and read the running bridge's parameters, not
just YAML defaults.

Conclusion: during this window, intermittent close returns, not malformed or
stale scans, reproduced the guard condition. This does not establish safe
target tracking, identify self returns, or authorize actual movement.

Temporary diagnostic paths on GOS:
`/home/user/m20_ws/m20_guard_readonly.py` and
`/home/user/m20_ws/run_m20_guard_readonly.sh`.
The diagnostic is bounded and exits; the existing dry-run launch remains running.
