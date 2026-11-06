import json
import queue
import socket
import threading
import time
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

from m20_adapter.core import Guard
from m20_adapter.operator_control import OperatorControl
from m20_adapter.operator_server import OperatorServer


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.control = OperatorControl(Guard(), preview=True)
        self.server = OperatorServer('a' * 40, b'<title>M20</title>', '127.0.0.1', 0, 0)
        # Bind an ephemeral UDP port for tests without reserving robot ports.
        self.server.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.udp.bind(('127.0.0.1', 0))
        self.server.udp.settimeout(.1)
        self.server.start()
        self.url = 'http://127.0.0.1:' + str(self.server.http.server_port)
        self.running = True

        def tick():
            while self.running:
                self.control.guard.update_scan(True, True)
                self.server.drain(self.control)
                self.control.tick()
                time.sleep(.01)

        self.worker = threading.Thread(target=tick)
        self.worker.start()

    def tearDown(self):
        self.running = False
        self.worker.join()
        self.server.close()

    def request(self, body=None, token='a' * 40, path='/api/command', origin=None):
        headers = {'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = origin
        req = urllib.request.Request(self.url + path,
                                     data=json.dumps(body).encode() if body is not None else None,
                                     headers=headers)
        with urllib.request.urlopen(req, timeout=2) as response:
            return json.load(response)

    def test_http_auth_origin_start_stop(self):
        command = dict(client='phone', seq=1, op='start', mode='direct')
        for kwargs in (dict(token='wrong'), dict(origin='http://evil.invalid')):
            with self.assertRaises(urllib.error.HTTPError):
                self.request(command, **kwargs)
        self.assertIsNone(self.control.owner)
        self.assertTrue(self.request(command)['ok'])
        self.assertEqual(self.control.owner, 'web:phone')
        self.assertFalse(self.control.guard.armed)  # preview is never real arm
        self.request(dict(client='phone', seq=2, op='stop'))
        self.assertIsNone(self.control.owner)

    def test_android_and_web_share_exclusive_ownership(self):
        self.request(dict(client='phone', seq=1, op='start', mode='direct'))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
            client.settimeout(2)
            addr = self.server.udp.getsockname()
            packet = dict(token='a' * 40, client='app', seq=1, op='start', mode='direct')
            client.sendto(json.dumps(packet).encode(), addr)
            reply = json.loads(client.recv(4096))
            self.assertFalse(reply['ok'])
            self.assertIn('another operator', reply['error'])
            packet.update(seq=2, op='stop')
            client.sendto(json.dumps(packet).encode(), addr)
            self.assertTrue(json.loads(client.recv(4096))['ok'])
        self.assertIsNone(self.control.owner)

    def test_expired_queue_cannot_arm(self):
        self.running = False
        self.worker.join()
        result = queue.Queue()
        self.server.commands.put((time.monotonic() - 1,
                                  dict(client='phone', seq=1, op='start', mode='direct'),
                                  'web', result))
        self.server.drain(self.control)
        self.assertFalse(result.get()['ok'])
        self.assertIsNone(self.control.owner)

    def test_status_requires_authentication(self):
        with self.assertRaises(urllib.error.HTTPError):
            self.request(path='/api/status', token='')
        self.assertIn('ready', self.request(path='/api/status'))

    def test_android_decoder_recovers_from_deep_json(self):
        original = json.loads
        decoded = threading.Event()

        def parser(value, *args, **kwargs):
            if value == b'nested-input':
                decoded.set()
                raise RecursionError('nested JSON')
            return original(value, *args, **kwargs)

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
            client.settimeout(2)
            with patch('m20_adapter.operator_server.json.loads', side_effect=parser):
                client.sendto(b'nested-input', self.server.udp.getsockname())
                self.assertTrue(decoded.wait(1))
                request = dict(token='a' * 40, client='app', seq=1, op='stop')
                client.sendto(json.dumps(request).encode(), self.server.udp.getsockname())
                self.assertTrue(json.loads(client.recv(4096))['ok'])


if __name__ == '__main__':
    unittest.main()
