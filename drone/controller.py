"""Drone controller thread: the bridge between the GARUDA OPS dashboard and the
simulator.

Each tick (~10 Hz):
  1. apply queued dashboard commands (takeoff, land, modes, sticks, waypoints, tuning)
  2. read drone state + Scene/Depth images from the backend
  3. clean depth (report §5.4) -> detect_obstacle_regions() (report §3.8)
  4. run detection, geolocate detections, update crowd/obstacle map layers
  5. decide the action for the active mode (AvoidancePlanner = report rules)
  6. send the velocity command; build requested views; raise incidents; record
"""
import collections
import datetime as _dt
import hashlib
import platform
import json
import math
import queue
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from . import vision
from .avoidance import AvoidancePlanner, detect_obstacle_regions, preprocess_depth, zone_label

MODES = ('IDLE', 'TAKEOFF', 'HOLD', 'MANUAL', 'AUTONOMOUS', 'GOTO', 'PATROL', 'RTH', 'LANDING')
FLYING = ('HOLD', 'MANUAL', 'AUTONOMOUS', 'GOTO', 'PATROL', 'RTH')
VIEWS = ('detection', 'depth', 'crowd', 'thermal', 'edges', 'quad')

DEFAULT_CFG = dict(
    # --- report parameters (Section 3.8) ---
    forward_speed=2.0,            # report used 1.0 m/s; raised for the demo, adjustable in the UI
    avoid_speed_sideways=1.0,
    avoid_duration=1.5,
    look_ahead_threshold=5.0,
    roi_height_ratio=0.6,
    altitude=5.0,                 # report: moveToZAsync(-5)
    # --- additions ---
    bias_duration=0.8,            # §5.2.6 forward-progress bias
    hard_stop=1.5,                # never move toward anything closer than this
    manual_speed=3.0,
    climb_rate=1.5,
    yaw_rate_max=45.0,
    assist=True,                  # obstacle assist in MANUAL (blocks moves into obstacles)
    ground_reject=True,
    median_filter=True,
    max_range=30.0,
    ema=0.6,                      # low-pass on sector distances (§5.4)
    waypoint_radius=2.5,
    crowd_moderate=6, crowd_high=12, crowd_very_high=20,
    crowd_range=45.0,             # only persons within this distance count toward crowd level
    # --- emergency response (police alert) ---
    alert_enabled=True,           # auto-prompt the operator when the crowd limit is exceeded
    alert_limit=12,               # persons near the drone that count as "too much"
    alert_hold_sec=3.0,           # limit must be exceeded continuously this long
    alert_cooldown_sec=60.0,      # after a decision, do not re-prompt for the same area this soon
    canny_lo=100, canny_hi=200,   # report values; lower them for low-contrast scenes
    crowd_heat_saturation=4.0,
    thermal_min_c=10.0, thermal_max_c=40.0, heat_alert_c=33.0,
)
NUMERIC = {k for k, v in DEFAULT_CFG.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
LIMITS = dict(alert_limit=(1, 200), forward_speed=(0.2, 8), avoid_speed_sideways=(0.2, 4), avoid_duration=(0.3, 5),
              look_ahead_threshold=(1, 20), roi_height_ratio=(0.2, 1.0), altitude=(1.5, 60),
              manual_speed=(0.2, 10), hard_stop=(0.5, 5), bias_duration=(0, 3))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def zone_of(x, y):
    return ('SECTOR ALPHA' if x >= 0 and y >= 0 else 'SECTOR BRAVO' if x >= 0 else
            'SECTOR CHARLIE' if y < 0 else 'SECTOR DELTA')


class DroneController(threading.Thread):
    def __init__(self, backend, detector, root, patrol=None, rate_hz=10.0):
        super().__init__(daemon=True, name='drone-controller')
        self.b, self.det, self.root = backend, detector, Path(root)
        self.cfg = dict(DEFAULT_CFG)
        self.period = 1.0 / rate_hz
        self.q = queue.Queue()
        self.lock = threading.Lock()
        self.frame_cond = threading.Condition()
        self.frame_id = 0
        self.jpeg = {}                    # view -> jpeg bytes
        self.view_wanted = {v: 0.0 for v in VIEWS}
        self.running = True
        self.planner = AvoidancePlanner()
        self.mode = 'IDLE'
        self.decision = 'IDLE — awaiting take-off'
        self.stick = dict(fwd=0.0, right=0.0, up=0.0, yaw=0.0, t=0.0)
        self.z_target = -self.cfg['altitude']
        self.target = None
        self.patrol = patrol or [(25, 0), (45, 22), (70, 45), (45, 70), (5, 45), (-15, 10)]
        self.patrol_idx = 0
        self.home = (0.0, 0.0)
        self.t_mode = time.time()
        self.t_start = time.time()
        self.st = dict(x=0, y=0, z=0, vx=0, vy=0, vz=0, yaw=0, pitch=0, roll=0, lat=0, lon=0, landed=True, collided=None)
        self.obs = dict(left=False, center=False, right=False, min_dist_left=np.inf,
                        min_dist_center=np.inf, min_dist_right=np.inf)
        self._ema = None
        self.dets = []
        self.obst = []
        self.trail = collections.deque(maxlen=600)
        self.occ = {}                      # (ix,iy) -> hits    (2 m cells)
        self.crowd = {}                    # (ix,iy) -> weight  (4 m cells)
        self.events = []
        self.event_id = 0
        self._last_evt = {}
        self.log = collections.deque(maxlen=80)
        self.series = collections.deque(maxlen=600)   # (t, clearance, persons)
        self.fps = 0.0
        self.loop_ms = 0.0
        self.assist_note = ''
        self.collided_flag = None
        self.error = None
        self.rec = None
        self._still = None
        self.tick_no = 0
        self.last_manifest = None
        self.alerts = []                  # every police alert (pending / sent / declined)
        self.alert_seq = 0
        self._over_since = None
        self._last_decision = None        # (time, x, y) of the last operator decision
        self._alert_snap_pending = None
        self.log_line('INFO', f'Backend: {backend.name} · Detector: {detector.label}')

    # ----------------------------------------------------------- public API
    def command(self, cmd):
        if not isinstance(cmd, dict) or 'action' not in cmd:
            raise ValueError('command needs an "action"')
        if cmd['action'] == 'stick':                       # fast path, no queue latency
            with self.lock:
                for k in ('fwd', 'right', 'up', 'yaw'):
                    self.stick[k] = float(np.clip(float(cmd.get(k, 0.0)), -1, 1))
                self.stick['t'] = time.time()
                if self.mode in ('AUTONOMOUS', 'GOTO', 'PATROL', 'RTH', 'HOLD') and any(
                        abs(self.stick[k]) > 0.05 for k in ('fwd', 'right', 'up', 'yaw')):
                    self.q.put(dict(action='mode', mode='MANUAL', reason='operator stick override'))
            return
        self.q.put(cmd)

    def want(self, view):
        if view in self.view_wanted:
            self.view_wanted[view] = time.time()

    def telemetry(self, since=0):
        with self.lock:
            st = self.st
            speed = math.hypot(st['vx'], st['vy'])
            persons = [d for d in self.dets if d['label'] == 'person']
            near = self._near_persons(self.dets)
            vehicles = [d for d in self.dets if d['label'] in ('car', 'bus', 'truck', 'motorcycle')]
            o = self.obs
            return dict(
                t=round(time.time() - self.t_start, 2),
                backend=self.b.name, detector=self.det.label, error=self.error,
                mode=self.mode, decision=self.decision, assist=self.assist_note,
                armed=self.mode != 'IDLE', collided=self.collided_flag,
                pos=dict(x=round(float(st['x']), 2), y=round(float(st['y']), 2), alt=round(-float(st['z']), 2)),
                vel=dict(vx=round(float(st['vx']), 2), vy=round(float(st['vy']), 2), vz=round(float(st['vz']), 2), speed=round(float(speed), 2)),
                heading=round(math.degrees(st['yaw']) % 360, 1),
                gps=dict(lat=round(st['lat'], 7), lon=round(st['lon'], 7)),
                zone=zone_of(st['x'], st['y']),
                sectors=dict(left=bool(o['left']), center=bool(o['center']), right=bool(o['right']),
                             dl=_f(o['min_dist_left']), dc=_f(o['min_dist_center']), dr=_f(o['min_dist_right']),
                             code=zone_label(o)),
                people=len(persons), vehicles=len(vehicles),
                people_near=near,
                crowd_level=vision.crowd_level(near, self.cfg),
                detections=[dict(label=d['label'], conf=d['conf'], dist=d.get('dist'), src=d['source'])
                            for d in sorted(self.dets, key=lambda d: d.get('dist', 99))[:14]],
                obstacles=[round(ob['dist'], 1) for ob in self.obst],
                target=self.target, patrol=self.patrol, patrol_idx=self.patrol_idx, home=self.home,
                trail=list(self.trail)[-300:],
                occ=[[k[0] * 2 + 1, k[1] * 2 + 1, min(9, v)] for k, v in list(self.occ.items())[-2500:] if v >= 2],
                crowd=[[k[0] * 4 + 2, k[1] * 4 + 2, round(v, 2)] for k, v in self.crowd.items() if v > 0.15],
                events=[e for e in self.events if e['seq'] > since][-50:],
                last_seq=self.event_id,
                log=list(self.log)[-30:],
                series=list(self.series)[-240:],
                fps=round(self.fps, 1), loop_ms=round(self.loop_ms, 1),
                cfg=self.cfg,
                recording=self.rec.info() if self.rec else None,
                last_manifest=self.last_manifest,
                alerts=[{k: v for k, v in al.items() if k != 'snapshot'} for al in self.alerts[-20:]],
                pending_alert=next(({k: v for k, v in al.items() if k != 'snapshot'}
                                    for al in self.alerts if al['status'] == 'PENDING'), None),
            )

    def stop(self):
        self.running = False

    # ----------------------------------------------------------- helpers
    def log_line(self, lvl, msg):
        self.log.append([time.strftime('%H:%M:%S'), lvl, msg])
        print(f'[{lvl}] {msg}')

    def event(self, etype, sev, desc, key=None, every=0.0):
        now = time.time()
        k = key or etype
        if every and now - self._last_evt.get(k, 0) < every:
            return
        self._last_evt[k] = now
        self.event_id += 1
        st = self.st
        e = dict(seq=self.event_id, id=f'EVT-{self.event_id:03d}',
                 time_sec=round(now - self.t_start, 1), type=etype, severity=sev,
                 x=round(st['x'], 1), y=round(st['y'], 1),
                 latitude=round(st['lat'], 7), longitude=round(st['lon'], 7),
                 zone=zone_of(st['x'], st['y']), description=desc)
        self.events.append(e)
        self.events = self.events[-300:]
        if self.rec:
            self.rec.incident(e)
        self.log_line('EVENT', f'{etype} [{sev}] — {desc}')

    def set_mode(self, m, why=''):
        if m == self.mode:
            return
        self.log_line('MODE', f'{self.mode} → {m}' + (f' ({why})' if why else ''))
        self.mode = m
        self.t_mode = time.time()
        self.planner.reset()

    # ----------------------------------------------------------- commands
    def _apply(self, c):
        a = c.get('action')
        airborne = self.mode != 'IDLE'
        if a == 'takeoff':
            if self.mode in ('IDLE', 'LANDING'):
                self.b.arm_takeoff()
                self.collided_flag = None
                self.home = (round(self.st['x'], 2), round(self.st['y'], 2))
                self.z_target = -self.cfg['altitude']
                self.set_mode('TAKEOFF', 'operator')
                self.event('Take-off', 'INFO', f"Climbing to {self.cfg['altitude']:.1f} m", every=1)
        elif a == 'land':
            self.b.land()
            self.set_mode('LANDING', 'operator')
            self.event('Landing initiated', 'INFO', 'Operator commanded landing', every=1)
        elif a in ('hold', 'estop'):
            self.b.hover()
            self.set_mode('HOLD', 'EMERGENCY STOP' if a == 'estop' else 'operator')
            if a == 'estop':
                self.event('Emergency stop', 'HIGH', 'Operator pressed E-STOP — drone holding position', every=1)
        elif a == 'mode':
            m = str(c.get('mode', '')).upper()
            if m not in MODES or m in ('IDLE', 'TAKEOFF', 'LANDING'):
                raise ValueError(f'bad mode {m}')
            if not airborne:
                self.log_line('WARN', f'{m} ignored — drone is on the ground (take off first)')
                return
            if m == 'PATROL':
                self.patrol_idx = self._nearest_patrol_idx()
                self.target = list(self.patrol[self.patrol_idx])
            elif m == 'RTH':
                self.target = list(self.home)
            elif m == 'GOTO' and not self.target:
                return
            elif m in ('MANUAL', 'AUTONOMOUS', 'HOLD'):
                self.target = None
            self.set_mode(m, c.get('reason', 'operator'))
        elif a == 'goto':
            x, y = float(c['x']), float(c['y'])
            self.target = [round(x, 1), round(y, 1)]
            if airborne:
                self.set_mode('GOTO', 'map waypoint')
            self.log_line('NAV', f'Waypoint set to ({x:.1f}, {y:.1f})')
        elif a == 'patrol_set':
            pts = [(float(p[0]), float(p[1])) for p in c.get('points', [])][:30]
            if len(pts) >= 2:
                self.patrol = pts
                self.patrol_idx = 0
                self.log_line('NAV', f'Patrol route updated ({len(pts)} waypoints)')
        elif a == 'set':
            for k, v in (c.get('params') or {}).items():
                if k not in self.cfg:
                    continue
                if isinstance(self.cfg[k], bool):
                    self.cfg[k] = bool(v)
                elif k in NUMERIC:
                    v = float(v)
                    lo, hi = LIMITS.get(k, (-1e9, 1e9))
                    self.cfg[k] = max(lo, min(hi, v))
                if k == 'altitude' and airborne:
                    self.z_target = -self.cfg['altitude']
            self.log_line('TUNE', ', '.join(f'{k}={self.cfg.get(k)}' for k in (c.get('params') or {})))
        elif a == 'reset':
            self.b.reset()
            self.planner.reset()
            self.trail.clear(); self.occ.clear(); self.crowd.clear()
            self.collided_flag = None
            self.set_mode('IDLE', 'simulation reset')
        elif a == 'alert_decision':
            self._decide_alert(str(c.get('id')), str(c.get('decision')))
        elif a == 'alert_test':
            persons = self._near_persons(self.dets)
            self._raise_alert(self.st, max(persons, self.cfg['alert_limit']), 'TEST', test=True)
        elif a == 'clear_map':
            self.occ.clear(); self.crowd.clear(); self.trail.clear()
        elif a == 'record':
            if c.get('state') == 'start' and not self.rec:
                self.rec = Recorder(self.root, self.b.name, self.t_start)
                self.log_line('REC', f'Recording → {self.rec.base}')
            elif c.get('state') == 'stop' and self.rec:
                path = self.rec.close(self.events)
                m = self.rec.manifest
                self.last_manifest = dict(name=self.rec.base, video=m.get('video_file'),
                                          video_sha256=m.get('video_sha256'), frames=m.get('source_frames'),
                                          start=m.get('start_time'), end=m.get('end_time'))
                self.log_line('REC', f'Saved session {path}')
                if m.get('video_sha256'):
                    self.log_line('REC', f"Video SHA-256 {m['video_sha256']}")
                self.log_line('REC', f'Integrity manifest {self.rec.base}.manifest.json — '
                                     f'verify with: python tools/verify_recording.py {self.rec.base}')
                self.rec = None
        else:
            raise ValueError(f'unknown action {a}')

    def _nearest_patrol_idx(self):
        x, y = self.st['x'], self.st['y']
        return int(np.argmin([math.hypot(px - x, py - y) for px, py in self.patrol]))

    # ----------------------------------------------------------- main loop
    def run(self):
        t_fps = time.time()
        n = 0
        while self.running:
            t0 = time.time()
            try:
                self._tick()
                self.error = None
            except Exception as e:  # keep the bridge alive, surface error to UI
                self.error = f'{type(e).__name__}: {e}'
                self.log_line('ERROR', self.error)
                time.sleep(0.5)
            n += 1
            self.loop_ms = (time.time() - t0) * 1000
            if time.time() - t_fps >= 1.0:
                self.fps = n / (time.time() - t_fps)
                t_fps, n = time.time(), 0
            time.sleep(max(0.0, self.period - (time.time() - t0)))
        if self.rec:
            self.rec.close(self.events)
        try:                                   # original script: finally -> landAsync, disarm
            if self.mode != 'IDLE':
                self.b.land()
                t_end = time.time() + 15
                while time.time() < t_end and not self.b.state()['landed']:
                    time.sleep(0.2)
                self.b.disarm()
        except Exception as e:
            print('[shutdown] landing failed:', e)
        self.b.close()

    def _tick(self):
        while True:
            try:
                c = self.q.get_nowait()
            except queue.Empty:
                break
            try:
                self._apply(c)
            except Exception as e:
                self.log_line('WARN', f'command {c.get("action")} rejected: {e}')

        st = self.b.state()
        imgs = self.b.images()
        if imgs is None:
            return
        cap_dt = _dt.datetime.now().astimezone()      # capture time used for the evidence stamp
        self.tick_no += 1
        cfg = self.cfg
        scene, depth = imgs['scene'], imgs['depth']
        if depth.shape != scene.shape[:2]:
            depth = cv2.resize(depth, (scene.shape[1], scene.shape[0]), interpolation=cv2.INTER_NEAREST)
        alt = -st['z']
        dproc = preprocess_depth(depth, cfg['max_range'], cfg['median_filter'], cfg['ground_reject'],
                                 altitude=alt, hfov_deg=self.b.hfov_deg, pitch_rad=st.get('pitch', 0.0))
        raw_obs = detect_obstacle_regions(dproc, threshold=cfg['look_ahead_threshold'],
                                          roi_height_ratio=cfg['roi_height_ratio'])
        obs = self._smooth(raw_obs, cfg)
        roi_start = int(depth.shape[0] * (1.0 - cfg['roi_height_ratio']))
        dets = vision.geolocate(self.det.detect(scene), depth, st, self.b.hfov_deg)
        obst = vision.obstacle_boxes(dproc, near=max(8.0, cfg['look_ahead_threshold'] * 1.6), roi_start=roi_start)

        # ---- state bookkeeping
        with self.lock:
            self.st, self.obs, self.dets, self.obst = st, obs, dets, obst
        if st['collided'] and not self.collided_flag and alt > 0.3:
            self.collided_flag = str(st['collided'])
            self.event('Collision detected', 'CRITICAL', f'Contact with {st["collided"]}', every=3)
            self.b.hover()
            self.set_mode('HOLD', 'collision')
        now = time.time()
        if not self.trail or math.hypot(st['x'] - self.trail[-1][0], st['y'] - self.trail[-1][1]) > 0.6:
            self.trail.append([round(st['x'], 1), round(st['y'], 1)])
        self._update_maps(st, depth, dets)
        persons = self._near_persons(dets)
        level = vision.crowd_level(persons, cfg)
        self.series.append([round(now - self.t_start, 1), _f(obs['min_dist_center'], 40), persons])
        if level in ('HIGH', 'VERY HIGH') and self.mode != 'IDLE':
            self.event('Crowd congestion', 'HIGH' if level == 'HIGH' else 'CRITICAL',
                       f'{persons} persons in view near {zone_of(st["x"], st["y"])} — operator review requested',
                       key='crowd', every=20)
        self._check_crowd_alert(st, persons, level, now)
        if self.mode in FLYING and _f(obs['min_dist_center'], 99) < cfg['hard_stop']:
            self.event('Low clearance', 'HIGH', f'Obstacle {obs["min_dist_center"]:.1f} m ahead', key='clear', every=6)

        # ---- control
        self._control(st, obs, now)

        # ---- views
        views = {}
        need = {v for v, t in self.view_wanted.items() if now - t < 3.0}
        if self.rec:
            need.add('detection')
        if self._alert_snap_pending:
            need.add('crowd')
        if 'quad' in need:
            need |= {'detection', 'depth', 'crowd', 'thermal'}
        if 'detection' in need:
            views['detection'] = vision.detection_view(scene, dets, obst, obs, cfg, self.decision, roi_start)
        if 'depth' in need:
            views['depth'] = vision.depth_view(depth, dproc, obs, cfg, roi_start, cfg['max_range'])
        if 'crowd' in need:
            views['crowd'] = vision.crowd_view(scene, dets, cfg, level)
        if 'thermal' in need:
            if 'thermal_c' in imgs:
                temp, src = imgs['thermal_c'], 'SIMULATED THERMAL (MOCK GROUND TRUTH)'
            else:
                temp, src = vision.synth_thermal(scene, depth, dets, imgs.get('infrared'))
            views['thermal'] = vision.thermal_view(temp, src, cfg)
        if 'edges' in need:
            views['edges'] = vision.edges_view(scene, roi_start, cfg['canny_lo'], cfg['canny_hi'])
        if 'quad' in need:
            views['quad'] = vision.quad_view(views)
        # evidence strip on every view: capture time (ms), recording clock, frame number
        src = 'AIRSIM' if self.b.name.startswith('AIRSIM') else 'MOCK SIM'
        if self.rec:
            rec_el = (cap_dt.timestamp() - self.rec.t_begin)
            fno = self.rec.src_frames + 1
        else:
            rec_el, fno = None, self.tick_no
        for k in list(views):
            views[k] = vision.stamp(views[k], cap_dt, fno, src, rec_el)
        if self._alert_snap_pending and 'crowd' in views:
            self._save_alert_snapshot(self._alert_snap_pending, views['crowd'])
            self._alert_snap_pending = None
        enc = {}
        for k, v in views.items():
            ok, buf = cv2.imencode('.jpg', v, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                enc[k] = buf.tobytes()
        if self.rec and 'detection' in views:
            self.rec.frame(views['detection'], self._observation(st, obs, persons, level, dets), cap_dt)
        with self.frame_cond:
            self.jpeg.update(enc)
            self.frame_id += 1
            self.frame_cond.notify_all()

    # ----------------------------------------------------------- emergency response
    def _check_crowd_alert(self, st, persons, level, now):
        cfg = self.cfg
        if not cfg['alert_enabled'] or self.mode == 'IDLE' or persons < cfg['alert_limit']:
            self._over_since = None
            return
        self._over_since = self._over_since or now
        if now - self._over_since < cfg['alert_hold_sec']:
            return
        if any(a['status'] == 'PENDING' for a in self.alerts):
            return
        if self._last_decision:
            t, x, y = self._last_decision
            # same area, decided recently -> do not nag the operator
            if now - t < cfg['alert_cooldown_sec'] and math.hypot(st['x'] - x, st['y'] - y) < 40:
                return
        self._raise_alert(st, persons, level)

    def _raise_alert(self, st, persons, level, test=False):
        if any(a['status'] == 'PENDING' for a in self.alerts):
            return
        self.alert_seq += 1
        dt = _dt.datetime.now().astimezone()
        al = dict(id=f'ALERT-{self.alert_seq:03d}', test=test, status='PENDING',
                  created=vision.fmt_ts(dt), created_epoch=dt.timestamp(),
                  persons=int(persons), limit=int(self.cfg['alert_limit']), crowd_level=level,
                  zone=zone_of(st['x'], st['y']),
                  x=round(float(st['x']), 1), y=round(float(st['y']), 1), alt=round(-float(st['z']), 1),
                  latitude=round(float(st['lat']), 7), longitude=round(float(st['lon']), 7),
                  station='Nearest police station', decided=None, snapshot=None, snapshot_sha256=None,
                  recording=self.rec.base if self.rec else None)
        self.alerts.append(al)
        self.alerts = self.alerts[-100:]
        self._alert_snap_pending = al['id']
        self.event('Crowd limit exceeded' + (' (TEST)' if test else ''), 'CRITICAL',
                   f"{al['persons']} persons near {al['zone']} (limit {al['limit']}). "
                   f"{al['id']} awaiting operator decision", every=0)

    def _save_alert_snapshot(self, alert_id, img):
        al = next((a for a in self.alerts if a['id'] == alert_id), None)
        if not al:
            return
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            return
        data = buf.tobytes()
        al['snapshot'] = data
        al['snapshot_sha256'] = hashlib.sha256(data).hexdigest()
        d = self.root / 'data' / 'alerts'
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{al['id']}_{time.strftime('%Y%m%d_%H%M%S')}.jpg").write_bytes(data)

    def _decide_alert(self, alert_id, decision):
        al = next((a for a in self.alerts if a['id'] == alert_id), None)
        if not al or al['status'] != 'PENDING':
            raise ValueError('no pending alert with that id')
        dt = _dt.datetime.now().astimezone()
        al['decided'] = vision.fmt_ts(dt)
        al['response_sec'] = round(dt.timestamp() - al['created_epoch'], 1)
        self._last_decision = (time.time(), self.st['x'], self.st['y'])
        tag = ' (TEST)' if al['test'] else ''
        if decision == 'send':
            al['status'] = 'SENT'
            self.event('Police alert sent' + tag, 'CRITICAL',
                       f"{al['id']} → {al['station']}: {al['persons']} persons at {al['zone']}, "
                       f"GPS {al['latitude']:.6f}, {al['longitude']:.6f}", every=0)
        else:
            al['status'] = 'DECLINED'
            self.event('Police alert declined' + tag, 'MEDIUM',
                       f"{al['id']} declined by operator ({al['persons']} persons at {al['zone']})", every=0)
        self._save_alert_log()

    def _save_alert_log(self):
        d = self.root / 'data' / 'alerts'
        d.mkdir(parents=True, exist_ok=True)
        log = [{k: v for k, v in a.items() if k != 'snapshot'} for a in self.alerts]
        (d / 'alerts_log.json').write_text(json.dumps(log, indent=1))

    def alert_snapshot(self, alert_id):
        al = next((a for a in self.alerts if a['id'] == alert_id), None)
        return al and al.get('snapshot')

    def _near_persons(self, dets):
        r = self.cfg['crowd_range']
        return sum(1 for d in dets if d['label'] == 'person' and (d.get('dist') or 0) <= r)

    def _smooth(self, o, cfg):
        a = cfg['ema']
        keys = ('min_dist_left', 'min_dist_center', 'min_dist_right')
        cur = {k: min(float(o[k]), 60.0) for k in keys}
        if self._ema is None:
            self._ema = cur
        else:
            # react instantly to things getting closer, smooth only when they recede
            self._ema = {k: cur[k] if cur[k] < self._ema[k] else a * self._ema[k] + (1 - a) * cur[k] for k in keys}
        th = cfg['look_ahead_threshold']
        out = {k: (np.inf if self._ema[k] >= 59.9 else self._ema[k]) for k in keys}
        out.update(left=out['min_dist_left'] < th, center=out['min_dist_center'] < th, right=out['min_dist_right'] < th)
        return out

    def _update_maps(self, st, depth, dets):
        # crowd density: decay then deposit each geolocated person
        for k in list(self.crowd):
            self.crowd[k] *= 0.985
            if self.crowd[k] < 0.05:
                del self.crowd[k]
        for d in dets:
            if d['label'] == 'person' and 'wx' in d:
                k = (int(math.floor(d['wx'] / 4)), int(math.floor(d['wy'] / 4)))
                self.crowd[k] = min(12.0, self.crowd.get(k, 0) + 0.12)
        # depth-mapped obstacles: per column, nearest obstacle in the ROI
        h, w = depth.shape
        alt = -st['z']
        roi = depth[int(h * 0.35):int(h * 0.75), ::8]
        if alt < 0.5 or roi.size == 0:
            return
        dmin = np.min(np.where(np.isfinite(roi), roi, np.inf), axis=0)
        f = (w / 2.0) / math.tan(math.radians(self.b.hfov_deg) / 2.0)
        cols = np.arange(0, w, 8)
        for c, d in zip(cols, dmin):
            if not np.isfinite(d) or d > 25 or d < 0.5:
                continue
            b = st['yaw'] + math.atan2(c - (w - 1) / 2, f)
            x, y = st['x'] + d * math.cos(b), st['y'] + d * math.sin(b)
            k = (int(math.floor(x / 2)), int(math.floor(y / 2)))
            self.occ[k] = self.occ.get(k, 0) + 1
        if len(self.occ) > 6000:
            for k in list(self.occ)[:1000]:
                del self.occ[k]

    def _heading_rate(self, st):
        if not self.target:
            return 0.0
        dx, dy = self.target[0] - st['x'], self.target[1] - st['y']
        err = math.degrees(wrap(math.atan2(dy, dx) - st['yaw']))
        return float(np.clip(1.6 * err, -self.cfg['yaw_rate_max'], self.cfg['yaw_rate_max']))

    def _send_body(self, st, vx_b, vy_b, yaw_rate):
        cy, sy = math.cos(st['yaw']), math.sin(st['yaw'])
        vx = vx_b * cy - vy_b * sy
        vy = vx_b * sy + vy_b * cy
        self.b.move(vx, vy, self.z_target, yaw_rate, duration=0.5)

    def _control(self, st, obs, now):
        cfg = self.cfg
        alt = -st['z']
        self.assist_note = ''
        m = self.mode
        if m == 'IDLE':
            # on the ground: never re-launch on our own — only TAKE OFF does that
            self.decision = 'ON GROUND — ready for take-off'
            return
        if m == 'TAKEOFF':
            self.decision = f'TAKING OFF → {cfg["altitude"]:.1f} m'
            if now - self.t_mode > 1.5:
                self.b.move(0, 0, self.z_target, 0, 0.5)
            if alt > 0.85 * cfg['altitude'] or now - self.t_mode > 12:
                self.set_mode('HOLD', 'reached altitude')
            return
        if m == 'LANDING':
            self.decision = 'LANDING'
            # touchdown = AirSim says landed, or no vertical motion for 1.5 s
            if abs(st['vz']) < 0.08 and now - self.t_mode > 2:
                self._still = self._still or now
            else:
                self._still = None
            if st['landed'] or (self._still and now - self._still > 1.5):
                self.b.disarm()          # as in the original script: land, then disarm
                self._still = None
                self.set_mode('IDLE', 'touchdown')
                self.event('Landed', 'INFO', 'Drone on the ground — motors disarmed', every=1)
            return
        if m == 'HOLD':
            self.decision = 'HOLDING POSITION' + (' — COLLISION' if self.collided_flag else '')
            self.b.move(0, 0, self.z_target, 0, 0.5)
            return
        if m == 'MANUAL':
            s = dict(self.stick)
            if now - s['t'] > 0.6:                 # dead-man: no input -> hover
                s.update(fwd=0, right=0, up=0, yaw=0)
            vx_b = s['fwd'] * cfg['manual_speed']
            vy_b = s['right'] * cfg['manual_speed']
            if s['up']:
                self.z_target = float(np.clip(self.z_target - s['up'] * cfg['climb_rate'] * self.period, -60, -1.0))
                cfg['altitude'] = round(-self.z_target, 1)
            if cfg['assist']:
                notes = []
                if vx_b > 0 and _f(obs['min_dist_center'], 99) < max(cfg['hard_stop'], cfg['look_ahead_threshold'] * 0.5):
                    vx_b = 0.0
                    notes.append('FORWARD')
                if vy_b < 0 and _f(obs['min_dist_left'], 99) < cfg['hard_stop']:
                    vy_b = 0.0
                    notes.append('LEFT')
                if vy_b > 0 and _f(obs['min_dist_right'], 99) < cfg['hard_stop']:
                    vy_b = 0.0
                    notes.append('RIGHT')
                if notes:
                    self.assist_note = 'ASSIST BLOCKED ' + '/'.join(notes)
            self._send_body(st, vx_b, vy_b, s['yaw'] * cfg['yaw_rate_max'])
            self.decision = 'MANUAL CONTROL' + (f' · {self.assist_note}' if self.assist_note else '')
            return

        # ---- autonomous family: AUTONOMOUS / GOTO / PATROL / RTH
        if self.target:
            dist = math.hypot(self.target[0] - st['x'], self.target[1] - st['y'])
            if dist < cfg['waypoint_radius']:
                if m == 'PATROL':
                    self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol)
                    self.target = list(self.patrol[self.patrol_idx])
                    self.log_line('NAV', f'Patrol waypoint {self.patrol_idx + 1}/{len(self.patrol)}')
                elif m == 'RTH':
                    self.b.land()
                    self.set_mode('LANDING', 'home reached')
                    return
                else:
                    self.event('Waypoint reached', 'INFO', f'Arrived at {self.target}', every=1)
                    self.target = None
                    self.set_mode('HOLD', 'waypoint reached')
                    return
        hr = self._heading_rate(st)
        fwd_scale = 1.0
        if self.target:
            err = abs(hr) / 1.6
            if err > 60:
                fwd_scale = 0.0              # turn toward target before advancing
            dist = math.hypot(self.target[0] - st['x'], self.target[1] - st['y'])
            fwd_scale *= min(1.0, dist / 6.0 + 0.25)
        vx_b, vy_b, yaw_rate, label, evt = self.planner.step(obs, cfg, heading_rate=hr)
        if label.startswith('PATH CLEAR') or label.startswith('SLIGHT'):
            vx_b *= fwd_scale
        if evt:
            self.event(*evt, key='avoid', every=2.5)
        self._send_body(st, vx_b, vy_b, yaw_rate)
        tgt = f' → WP ({self.target[0]:.0f},{self.target[1]:.0f})' if self.target else ''
        self.decision = label + tgt

    def _observation(self, st, obs, persons, level, dets):
        return dict(person_count=persons,
                    vehicle_count=sum(1 for d in dets if d['label'] in ('car', 'bus', 'truck', 'motorcycle')),
                    crowd_level=level, latitude=round(st['lat'], 7), longitude=round(st['lon'], 7),
                    zone=zone_of(st['x'], st['y']), source='LIVE_' + self.b.name.replace(' ', '_'),
                    altitude_m=round(-st['z'], 2), heading_deg=round(math.degrees(st['yaw']) % 360, 1),
                    mode=self.mode, decision=self.decision,
                    min_dist_left=_f(obs['min_dist_left']), min_dist_center=_f(obs['min_dist_center']),
                    min_dist_right=_f(obs['min_dist_right']),
                    detections=[dict(label=d['label'], conf=d['conf'], dist=d.get('dist')) for d in dets[:20]])


def _f(v, cap=None):
    if v is None or not np.isfinite(v):
        return cap
    return round(float(v), 2)


# ====================================================================== recording
class Recorder:
    """Saves the detection-overlay video + a dataset in Vaishnav's GARUDA OPS
    schema, so a live AirSim flight can be replayed on the original dashboard."""
    FPS = 10

    def __init__(self, root, backend_name, t0):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.base = f'session_{stamp}'
        self.root = Path(root)
        self.backend = backend_name
        self.t_begin = time.time()
        self.writer = None
        self.video_name = None
        self.frames = 0
        self.obs = []
        self.incidents = []
        self._next_obs = 0.0
        self.src_frames = 0                  # unique stamped frames received
        self.chain = hashlib.sha256(b'GARUDA-OPS-EVIDENCE:' + self.base.encode()).hexdigest()
        self.frame_log = []                  # [frame_no, capture time] for every stamped frame
        self.first_ts = self.last_ts = None
        self.manifest = {}

    def _open(self, w, h):
        for fourcc, ext in (('avc1', '.mp4'), ('VP80', '.webm'), ('mp4v', '.mp4')):
            path = self.root / 'media' / (self.base + ext)
            wr = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), self.FPS, (w, h))
            if wr.isOpened():
                self.writer, self.video_name, self.codec = wr, path.name, fourcc
                return
            wr.release()
            try:
                path.unlink()
            except OSError:
                pass
        self.video_name, self.codec = None, None

    def frame(self, img, obs_row, cap_dt=None):
        t = time.time() - self.t_begin
        cap_dt = cap_dt or _dt.datetime.now().astimezone()
        self.src_frames += 1
        ts = vision.fmt_ts(cap_dt)
        self.first_ts = self.first_ts or ts
        self.last_ts = ts
        self.frame_log.append([self.src_frames, ts])
        # hash chain over every frame's pixels: any removed/reordered/edited frame breaks it
        self.chain = hashlib.sha256(self.chain.encode() + img.tobytes()).hexdigest()
        if self.writer is None and self.video_name is None and self.frames == 0:
            self._open(img.shape[1], img.shape[0])
        if self.writer is not None:
            due = int(t * self.FPS) + 1                 # keep video time == wall time
            while self.frames < due:
                self.writer.write(img)
                self.frames += 1
        else:
            self.frames += 1
        if t >= self._next_obs:
            self.obs.append(dict(time_sec=round(t, 1), **obs_row))
            self._next_obs += 2.0

    def incident(self, e):
        t = time.time() - self.t_begin
        self.incidents.append(dict(id=e['id'], time_sec=round(t, 1), type=e['type'], severity=e['severity'],
                                   latitude=e['latitude'], longitude=e['longitude'], zone=e['zone'],
                                   description=e['description']))

    def info(self):
        return dict(name=self.base, seconds=round(time.time() - self.t_begin, 1),
                    video=self.video_name, codec=getattr(self, 'codec', None))

    def close(self, events):
        if self.writer is not None:
            self.writer.release()
        dur = round(time.time() - self.t_begin, 1)
        live = self.backend.startswith('AIRSIM')
        data = dict(
            schema_version='1.0',
            name=f'Live flight {self.base}',
            origin=('RECORDED LIVE FROM AIRSIM via the GARUDA–DRDO bridge (depth-based avoidance + '
                    'detections). Counts come from the selected detector.') if live else
                   ('RECORDED FROM THE BUILT-IN MOCK SIMULATOR (synthetic test world) — not AirSim footage.'),
            video_filename=self.video_name, duration_sec=max(dur, 1),
            fps_sampling_interval_sec=2,
            coordinate_note=('AirSim GPS (geo-referenced from the simulator home point)' if live else
                             'Mock simulator local frame converted to illustrative lat/lon'),
            observations=self.obs or [dict(time_sec=0, person_count=0, vehicle_count=0, crowd_level='LOW',
                                           latitude=0, longitude=0, zone='N/A')],
            incidents=self.incidents)
        out = self.root / 'data' / (self.base + '.json')
        out.write_text(json.dumps(data, indent=2))
        video_path = self.root / 'media' / self.video_name if self.video_name else None
        self.manifest = dict(
            purpose='Tamper-evidence record for a GARUDA OPS drone recording. Recompute the SHA-256 '
                    'hashes of the files below; any change to the files makes them differ.',
            session=self.base,
            start_time=self.first_ts, end_time=self.last_ts, duration_sec=dur,
            source_frames=self.src_frames, video_frames_written=self.frames,
            video_fps=self.FPS, codec=getattr(self, 'codec', None),
            video_file=self.video_name,
            video_sha256=sha256_file(video_path) if video_path and video_path.is_file() else None,
            dataset_file=out.name, dataset_sha256=sha256_file(out),
            frame_chain_sha256=self.chain,
            frame_log=self.frame_log,
            backend=self.backend, host=platform.node(),
            generated_at=vision.fmt_ts(_dt.datetime.now().astimezone()))
        (self.root / 'data' / (self.base + '.manifest.json')).write_text(json.dumps(self.manifest, indent=1))
        return out.name


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


SESSION_RE = re.compile(r'^session_\d{8}_\d{6}$')
