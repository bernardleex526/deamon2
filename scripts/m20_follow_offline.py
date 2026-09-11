#!/usr/bin/env python3
"""Run production tracker and Guard without ROS discovery or any robot transport."""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from m20_adapter.core import Guard, ZERO, inspect_scan  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracker', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--follow-distance', type=float, default=2.0)
    args = parser.parse_args()
    if not math.isfinite(args.follow_distance) or args.follow_distance <= 0:
        parser.error('follow distance must be positive and finite')
    result = subprocess.run([str(args.tracker.resolve()), str(args.follow_distance)],
                            check=True, capture_output=True, text=True, timeout=30)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'tracker.csv').write_text(result.stdout)
    now = [0.0]
    guard = Guard(clock=lambda: now[0])
    unarmed = Guard(clock=lambda: now[0])
    status = dict(MotionState=17, Gait=12290, Charge=0, HES=0,
                  ControlUsageMode=1, Sleep=0, Direction=0)
    reports = []
    for row in csv.DictReader(io.StringIO(result.stdout)):
        t = float(row['time'])
        now[0] = t
        ranges = [math.inf if r is None else r for r in json.loads(row.pop('ranges'))]
        sec = int(t)
        scan = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((t-sec)*1e9))),
            ranges=ranges, angle_min=-math.pi, angle_increment=math.pi/360,
            range_min=.1, range_max=12.)
        valid, clear = inspect_scan(scan, t)
        raw = tuple(float(row[k]) for k in ('raw_x', 'raw_y', 'raw_yaw'))
        for gate in (guard, unarmed):
            gate.update_status(status)
            gate.update_scan(valid, clear)
        if not reports:
            assert guard.arm(), guard.reason  # In-memory test object, never the robot.
        guard.update_command(raw)
        unarmed.update_command(raw)
        output = guard.output()
        assert unarmed.output() == ZERO
        assert all(math.isfinite(v) for v in raw + output)
        assert all(abs(v) <= lim for v, lim in zip(output, guard.limits))
        if row['stage'] in ('target_lost', 'near_obstacle', 'empty_scan'):
            assert output == ZERO, row
        if row['stage'] in ('target_lost', 'empty_scan'):
            assert raw == ZERO, row
        if row['stage'] not in ('target_lost', 'empty_scan'):
            error = math.hypot(float(row['truth_x']) - float(row['target_x']),
                               float(row['truth_y']) - float(row['target_y']))
            assert int(row['target_points']) > 0 and error < .25, row
        if row['stage'] == 'hold_3m' and args.follow_distance < 2.5:
            assert output[0] > 0 and abs(raw[1]) < .02, row
        if row['stage'] in ('two_oclock', 'ten_oclock') and args.follow_distance <= 2:
            sign = -1 if row['stage'] == 'two_oclock' else 1
            assert abs(raw[1]) < .02 and output[1] == 0 and sign*output[2] > 0, row
        row.update(guard_x=output[0], guard_y=output[1], guard_yaw=output[2],
                   scan_valid=valid, scan_clear=clear, reason=guard.reason)
        reports.append(row)

    # Deterministic fault injection against the real Guard, not a replacement model.
    faults = {}
    for fault in ('scan_timeout', 'command_timeout', 'status_timeout', 'estop'):
        now[0] = 100.
        g = Guard(clock=lambda: now[0])
        g.update_status(status); g.update_scan(True, True)
        assert g.arm()
        g.update_command((.4, -.4, -.5))
        assert g.output() != ZERO
        if fault == 'estop':
            g.update_status(dict(status, HES=1))
        else:
            now[0] += 2.
            if fault != 'status_timeout': g.update_status(status)
            if fault != 'scan_timeout': g.update_scan(True, True)
            if fault != 'command_timeout': g.update_command((.4, -.4, -.5))
        assert g.output() == ZERO and not g.armed
        faults[fault] = g.reason
        g.update_status(status); g.update_scan(True, True)
        g.update_command((.4, -.4, -.5))
        assert g.output() == ZERO  # Recovery never auto-arms.

    with (args.output / 'guarded.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(reports[0]))
        writer.writeheader(); writer.writerows(reports)
    summary = dict(samples=len(reports), follow_distance=args.follow_distance,
                   simulated_gait=12290, faults=faults,
                   note='OFFLINE ONLY. No robot motion, identity proof or physics simulation.',
                   stages={})
    for stage in dict.fromkeys(r['stage'] for r in reports):
        r = next(r for r in reversed(reports) if r['stage'] == stage)
        summary['stages'][stage] = {k: r[k] for k in
            ('raw_x', 'raw_y', 'raw_yaw', 'guard_x', 'guard_y', 'guard_yaw', 'reason')}
    # Expose a real commissioning constraint: gait 4097 needs |vy|>=.35,
    # above the current .30 lateral cap. Never silently raise that cap.
    g = Guard(clock=lambda: now[0])
    g.update_status(dict(status, Gait=4097)); g.update_scan(True, True)
    assert g.arm()
    g.update_command((.4, -.5, -.6))
    summary['gait_4097_example'] = g.output()
    assert summary['gait_4097_example'][1] == 0
    summary['direction_note'] = ('Upstream forward-distance and yaw response retained. '
                                 'No target-directed lateral policy or turn-first state machine. '
                                 'Corridor/APF can still generate lateral correction.')
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        ts = [float(r['time']) for r in reports]
        for ax, axis in zip(axes, ('x', 'y', 'yaw')):
            ax.plot(ts, [float(r['raw_'+axis]) for r in reports], label='tracker raw')
            ax.plot(ts, [float(r['guard_'+axis]) for r in reports], label='simulated armed guard')
            ax.set_ylabel(axis + (' (rad/s)' if axis == 'yaw' else ' (m/s)'))
            ax.grid(True); ax.legend()
        axes[-1].set_xlabel('Scenario time (s)')
        fig.suptitle('M20 offline replay: 2m -> 3m -> right -> left -> loss -> faults')
        fig.tight_layout(); fig.savefig(args.output / 'velocity.png'); plt.close(fig)
    except ImportError:
        print('Plot skipped: matplotlib unavailable; CSV and JSON saved.')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
