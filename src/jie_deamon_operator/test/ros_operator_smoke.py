"""Real HTTP -> ROS tracker -> tagged preview smoke; no AOS transport."""
import json
import os
import secrets
import signal
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.executors import SingleThreadedExecutor
from m20_adapter.mock_inputs import MockInputs


def main():
    token = secrets.token_hex(24)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'token'
        path.write_text(token)
        process = subprocess.Popen([
            'ros2', 'launch', 'jie_deamon', 'm20.launch.py',
            'cloud_topic:=/m20/mock_points', 'operator_host:=127.0.0.1',
            'operator_port:=18080', 'android_port:=0',
            'operator_token_file:=' + str(path)], start_new_session=True)
        rclpy.init()
        mock = MockInputs()
        executor = SingleThreadedExecutor()
        executor.add_node(mock)
        seq = 0

        def http(body=None):
            nonlocal seq
            if body:
                seq += 1
                body = dict(client='smoke', seq=seq, **body)
            request = urllib.request.Request(
                'http://127.0.0.1:18080/api/' + ('command' if body else 'status'),
                data=json.dumps(body).encode() if body else None,
                headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=.5) as response:
                return json.load(response)

        def wait(predicate, heartbeat=False, timeout=15):
            end = time.monotonic() + timeout
            last_beat = 0.
            while time.monotonic() < end:
                assert process.poll() is None, 'launch exited'
                executor.spin_once(timeout_sec=.02)
                try:
                    status = http()
                    if heartbeat and time.monotonic() - last_beat > .1:
                        assert http(dict(op='heartbeat'))['ok']
                        last_beat = time.monotonic()
                    if predicate(status):
                        return status
                except (OSError, ValueError):
                    pass
            raise AssertionError('operator integration condition timed out')

        try:
            wait(lambda s: len(s.get('scan', [])) > 10)
            assert http(dict(op='start', mode='follow', target=[1.8, 0.]))['ok']
            status = wait(lambda s: s['follow_ready'] and s['operator']['raw_velocity'][0] > .2,
                          heartbeat=True)
            assert status['dry_run'] and not status['armed']
            assert status['velocity'] == [0., 0., 0.]
            # Keep sensor/algorithm alive, deliberately cease operator heartbeat.
            status = wait(lambda s: s['operator']['owner'] is None)
            assert status['velocity'] == [0., 0., 0.]
            print('PASS: HTTP target -> tagged follow preview; operator disconnect stops')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            executor.shutdown()
            mock.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
