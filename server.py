"""GARUDA OPS × DRDO Autonomous Obstacle Avoidance — integrated server.

    python server.py                 # auto: AirSim if running, else built-in mock simulator
    python server.py --sim airsim    # require AirSim (Unreal "Play" must be pressed first)
    python server.py --sim mock      # test without AirSim

Open http://localhost:8000  (live mission control)  ·  /replay  (Vaishnav's dataset/video replay)

Based on Vaishnav's GARUDA OPS server (stdlib HTTP server).  Added: the drone
controller thread, command/telemetry API, MJPEG sensor streams, recording and
session replay.
"""
import argparse
import json
import re
import sys
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent
CTRL = None
MEDIA_RE = re.compile(r'^[A-Za-z0-9_\-]+\.(mp4|webm)$')
SESSION_RE = re.compile(r'^session_\d{8}_\d{6}$')
STREAM_VIEWS = ('detection', 'depth', 'crowd', 'thermal', 'edges', 'quad')


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(ROOT / 'web'), **kw)

    def log_message(self, fmt, *args):     # keep the console readable
        if any(k in self.path for k in ('/api/telemetry', '/stream/', '/api/command')):
            return
        super().log_message(fmt, *args)

    # ------------------------------------------------------------ helpers
    def _json(self, obj, code=200):
        data = json.dumps(obj, default=_default).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        if n > 1_000_000:
            raise ValueError('body too large')
        return json.loads(self.rfile.read(n) or b'{}')

    # ------------------------------------------------------------ POST
    def do_POST(self):
        path = urlparse(self.path).path
        try:
            if path == '/api/command':
                CTRL.command(self._body())
                return self._json({'ok': True})
        except Exception as e:
            return self._json({'ok': False, 'error': str(e)}, 400)
        self.send_error(404)

    def do_HEAD(self):
        path = urlparse(self.path).path
        if path.startswith('/media/'):
            name = path.removeprefix('/media/')
            target = ROOT / 'media' / name
            if not MEDIA_RE.match(name) or not target.is_file():
                return self.send_error(404)
            self.send_response(200)
            self.send_header('Content-Type', 'video/webm' if name.endswith('.webm') else 'video/mp4')
            self.send_header('Accept-Ranges', 'bytes')
            self.send_header('Content-Length', str(target.stat().st_size))
            self.end_headers()
            return
        return super().do_HEAD()

    # ------------------------------------------------------------ GET
    def do_GET(self):
        u = urlparse(self.path)
        path, qs = u.path, parse_qs(u.query)
        if path == '/api/health':
            return self._json({'status': 'ok', 'mode': 'live bridge', 'backend': CTRL.b.name,
                               'detector': CTRL.det.label})
        if path == '/api/telemetry':
            since = int((qs.get('since') or ['0'])[0] or 0)
            return self._json(CTRL.telemetry(since))
        if path.startswith('/stream/'):
            return self._mjpeg(path.removeprefix('/stream/'))
        if path.startswith('/api/alert/') and path.endswith('.jpg'):
            data = CTRL.alert_snapshot(path.removeprefix('/api/alert/').removesuffix('.jpg'))
            if not data:
                return self.send_error(404)
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if path.startswith('/snapshot/'):
            view = path.removeprefix('/snapshot/').removesuffix('.jpg')
            return self._snapshot(view)
        if path == '/api/sessions':
            out = []
            for p in sorted((ROOT / 'data').glob('session_*.json'), reverse=True):
                if p.name.endswith('.manifest.json'):
                    continue
                try:
                    d = json.loads(p.read_text())
                    out.append(dict(name=p.stem, video=d.get('video_filename'), duration=d.get('duration_sec'),
                                    observations=len(d.get('observations', [])), incidents=len(d.get('incidents', []))))
                except Exception:
                    pass
            return self._json(out)
        if path == '/api/dataset':
            name = (qs.get('session') or [''])[0]
            if name:
                if not SESSION_RE.match(name):
                    return self.send_error(400)
                target = ROOT / 'data' / f'{name}.json'
            else:
                target = ROOT / 'data' / 'observations.json'
            if not target.is_file():
                return self.send_error(404)
            data = target.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if path.startswith('/media/'):
            return self._media(path.removeprefix('/media/'))
        if path == '/':
            self.path = '/index.html'
        elif path in ('/replay', '/replay/'):
            self.path = '/replay.html'
        return super().do_GET()

    def _snapshot(self, view):
        if view not in STREAM_VIEWS:
            return self.send_error(404)
        CTRL.want(view)
        deadline = time.time() + 3
        while view not in CTRL.jpeg and time.time() < deadline:
            time.sleep(0.05)
        data = CTRL.jpeg.get(view)
        if not data:
            return self.send_error(503)
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _mjpeg(self, view):
        if view not in STREAM_VIEWS:
            return self.send_error(404)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        last = -1
        try:
            while CTRL.running:
                CTRL.want(view)
                with CTRL.frame_cond:
                    CTRL.frame_cond.wait_for(lambda: CTRL.frame_id != last, timeout=1.0)
                    last = CTRL.frame_id
                    data = CTRL.jpeg.get(view)
                if not data:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                 str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _media(self, filename):
        # Vaishnav's byte-range video server, generalised to recorded sessions
        if not MEDIA_RE.match(filename):
            return self.send_error(404)
        target = ROOT / 'media' / filename
        if not target.is_file():
            return self.send_error(404)
        size = target.stat().st_size
        rng = self.headers.get('Range', '')
        start, end = 0, size - 1
        if rng.startswith('bytes='):
            try:
                a, b = rng[6:].split('-', 1)
                start = int(a) if a else 0
                end = int(b) if b else end
            except ValueError:
                return self.send_error(416)
        if start > end or start >= size:
            return self.send_error(416)
        end = min(end, size - 1)
        self.send_response(206 if rng else 200)
        self.send_header('Content-Type', 'video/webm' if filename.endswith('.webm') else 'video/mp4')
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Content-Length', str(end - start + 1))
        if rng:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.end_headers()
        try:
            with target.open('rb') as f:
                f.seek(start)
                remaining = end - start + 1
                while remaining:
                    chunk = f.read(min(65536, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass


def _default(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    return str(o)


def main():
    global CTRL
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sim', choices=('auto', 'airsim', 'mock'), default='auto')
    ap.add_argument('--airsim-host', default='127.0.0.1')
    ap.add_argument('--vehicle', default='', help='AirSim vehicle name (blank = default drone)')
    ap.add_argument('--camera', default='0', help='AirSim camera name/id (report used "0")')
    ap.add_argument('--detector', choices=('auto', 'yolo', 'airsim', 'sim', 'none'), default='auto')
    ap.add_argument('--yolo-weights', default='yolov8n.pt')
    ap.add_argument('--detect-meshes', default='*Person*,*person*,*Pedestrian*,*Character*,*Car*,*car*,*Vehicle*',
                    help='AirSim detection API mesh-name filters (comma separated, wildcards ok)')
    ap.add_argument('--infrared', action='store_true', help='use AirSim ImageType.Infrared for the thermal view')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8000)
    args = ap.parse_args()

    try:
        import cv2  # noqa
        import numpy  # noqa
    except ImportError:
        sys.exit('Missing packages. Run:  pip install -r requirements.txt')
    from drone.backends import make_backend
    from drone.controller import DroneController
    from drone.detector import Detector

    backend = make_backend(args.sim, args.airsim_host, args.vehicle, args.camera,
                           [m for m in args.detect_meshes.split(',') if m], args.infrared)
    detector = Detector(args.detector, backend, args.yolo_weights)
    CTRL = DroneController(backend, detector, ROOT)
    if backend.name.startswith('MOCK'):
        CTRL.cfg.update(canny_lo=40, canny_hi=120)     # flat-shaded mock textures need lower thresholds
    CTRL.start()
    (ROOT / 'media').mkdir(exist_ok=True)

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.daemon_threads = True
    print('=' * 64)
    print(f' GARUDA OPS × DRDO OBSTACLE AVOIDANCE  —  backend: {backend.name}')
    print(f' Mission control : http://localhost:{args.port}')
    print(f' Replay (Vaishnav): http://localhost:{args.port}/replay')
    print('=' * 64)

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down — landing the drone (as in the original script\'s finally: block)...')
    finally:
        CTRL.stop()
        CTRL.join(timeout=20)
        srv.server_close()


if __name__ == '__main__':
    main()
