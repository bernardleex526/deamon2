"""Token-authenticated LAN HTTP and Android JSON/UDP adapters.

Network threads never touch ROS or the motion guard. Commands are bounded,
timestamped and consumed by the bridge's single-threaded executor.
"""
import hmac
import json
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


class OperatorServer:
    def __init__(self, token, page, host='0.0.0.0', port=8080, android_port=8889):
        if len(token) < 32:
            raise ValueError('operator token must contain at least 32 characters')
        self.token = token
        self.page = page
        self.commands = queue.Queue(maxsize=32)
        self.snapshot = dict(ready=False, reason='waiting for bridge')
        self.lock = threading.Lock()
        self.running = False
        adapter = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(1.0)

            def log_message(self, *_):
                pass  # Never log credentials, query strings or request bodies.

            def reply(self, code, value, content_type='application/json'):
                data = value if isinstance(value, bytes) else json.dumps(value).encode()
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('X-Frame-Options', 'DENY')
                self.end_headers()
                self.wfile.write(data)

            def authorized(self):
                supplied = self.headers.get('Authorization', '')
                return hmac.compare_digest(supplied.encode(), ('Bearer ' + adapter.token).encode())

            def do_GET(self):
                path = urlsplit(self.path).path
                if path == '/':
                    self.reply(200, adapter.page, 'text/html; charset=utf-8')
                elif path == '/api/status' and self.authorized():
                    with adapter.lock:
                        snapshot = adapter.snapshot
                    self.reply(200, snapshot)
                else:
                    self.reply(401, dict(error='token required or unknown path'))

            def do_POST(self):
                if self.path != '/api/command' or not self.authorized():
                    self.reply(401, dict(error='token required'))
                    return
                # No CORS; reject cross-origin requests even with credentials.
                origin = self.headers.get('Origin')
                if origin and origin != 'http://' + self.headers.get('Host', ''):
                    self.reply(403, dict(error='cross-origin request rejected'))
                    return
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 4096:
                        raise ValueError('invalid request size')
                    message = json.loads(self.rfile.read(size))
                    if not isinstance(message, dict):
                        raise ValueError('JSON object required')
                    result = queue.Queue(maxsize=1)
                    adapter.commands.put_nowait((time.monotonic(), message, 'web', result))
                    response = result.get(timeout=1.0)
                    self.reply(200 if response['ok'] else 409, response)
                except (ValueError, RecursionError, queue.Full, queue.Empty, OSError) as exc:
                    self.reply(400, dict(error=str(exc) or 'gateway busy'))

        self.http = ThreadingHTTPServer((host, port), Handler)
        self.udp = None
        try:
            if android_port:
                self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp.bind((host, android_port))
                self.udp.settimeout(0.2)
        except Exception:
            self.http.server_close()
            if self.udp:
                self.udp.close()
            raise
        self.threads = []

    def start(self):
        self.running = True
        for target in (self.http.serve_forever, self.android_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self.threads.append(thread)

    def android_loop(self):
        while self.running and self.udp:
            try:
                data, address = self.udp.recvfrom(4097)
                if len(data) > 4096:
                    continue
                request = json.loads(data)
                if not isinstance(request, dict):
                    continue
                supplied = request.pop('token', '')
                if not isinstance(supplied, str) or not hmac.compare_digest(supplied.encode(), self.token.encode()):
                    continue
                result = queue.Queue(maxsize=1)
                self.commands.put_nowait((time.monotonic(), request, 'android', result))
                response = result.get(timeout=0.3)
                response['seq'] = request.get('seq')
                self.udp.sendto(json.dumps(response).encode(), address)
            except (ValueError, RecursionError, queue.Full, queue.Empty, OSError):
                continue

    def drain(self, control):
        for _ in range(32):
            try:
                received, request, source, result = self.commands.get_nowait()
            except queue.Empty:
                break
            try:
                if time.monotonic() - received > 0.25:
                    raise ValueError('expired request; retry with a new sequence')
                control.handle(request, source)
                result.put_nowait(dict(ok=True, **control.snapshot()))
            except (ValueError, TypeError) as exc:
                result.put_nowait(dict(ok=False, error=str(exc)))

    def update(self, value):
        with self.lock:
            self.snapshot = value

    def close(self):
        self.running = False
        self.http.shutdown()
        self.http.server_close()
        if self.udp:
            self.udp.close()
        for thread in self.threads:
            thread.join(timeout=2)
