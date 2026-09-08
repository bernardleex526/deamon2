#!/usr/bin/env python3
"""Read AOS telemetry using heartbeat only. Never send motion or mode commands."""
import argparse
import json
import time

from m20_adapter.core import Guard, UdpClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='10.21.31.103')
    parser.add_argument('--seconds', type=float, default=6.0)
    args = parser.parse_args()
    client = UdpClient(args.host)
    counts = {}
    status = None
    guard = Guard()
    last_heartbeat = -float('inf')
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            if time.monotonic() - last_heartbeat >= 1.0:
                client.heartbeat()
                last_heartbeat = time.monotonic()
            for _, value in client.receive():
                key = '{}:{}'.format(value['Type'], value['Command'])
                counts[key] = counts.get(key, 0) + 1
                if value['Type'] == 1002 and value['Command'] == 6:
                    status = value['Items']
                    if status.get('ErrorCode', 0) != 0:
                        guard.update_status({})
                    else:
                        guard.update_status(status)
            time.sleep(0.02)
    finally:
        client.close()
    fresh = bool(guard.status) and time.monotonic() - guard.status_at <= guard.status_timeout
    print(json.dumps(dict(sent_types=['100:100'], received=counts,
                         complete_fresh_status=fresh, last_status=status), indent=2))
    return 0 if fresh else 1


if __name__ == '__main__':
    raise SystemExit(main())
