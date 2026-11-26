"""Vision-based obstacle avoidance — from the DYSL-AT/DRDO internship project.

`detect_obstacle_regions` is the report's function (Section 3.8) unchanged.
`preprocess_depth` adds the Section 5.4 fixes (median filter, range cap) and an
optional ground-plane rejection so the floor is not mistaken for an obstacle at
low altitude. `AvoidancePlanner` is the report's rule set (Sections 3.6, 3.7,
5.2.4, 5.2.6) rewritten as a non-blocking state machine: the original loop used
`.join()` and `time.sleep()`, which would freeze the dashboard's telemetry.
"""
import math
import time

import numpy as np

try:
    import cv2
except ImportError:  # vision still works without the median filter
    cv2 = None


# ---------------------------------------------------------------- original
def detect_obstacle_regions(depth_img, threshold=3.0, roi_height_ratio=0.6):
    h, w = depth_img.shape
    roi_start_row = int(h * (1.0 - roi_height_ratio))
    roi_depth_img = depth_img[roi_start_row:, :]

    third = w // 3
    left_region = roi_depth_img[:, :third]
    center_region = roi_depth_img[:, third:2 * third]
    right_region = roi_depth_img[:, 2 * third:]

    def min_dist(region):
        if region.size == 0 or np.all(np.isinf(region)):
            return np.inf
        return np.min(region)

    return {
        'left': min_dist(left_region) < threshold,
        'center': min_dist(center_region) < threshold,
        'right': min_dist(right_region) < threshold,
        'min_dist_left': min_dist(left_region),
        'min_dist_center': min_dist(center_region),
        'min_dist_right': min_dist(right_region)
    }


# ---------------------------------------------------------------- Section 5.4
def preprocess_depth(depth, max_range=30.0, median=True, ground_reject=True,
                     altitude=5.0, hfov_deg=90.0, pitch_rad=0.0):
    """Clean a DepthPerspective image (metres).  Returns a new float32 array
    where 'no reading' / sky / ground / beyond max_range are np.inf."""
    d = np.array(depth, dtype=np.float32, copy=True)
    d[~np.isfinite(d)] = np.inf
    d[d <= 0] = np.inf
    if median and cv2 is not None:
        finite = np.isfinite(d)
        tmp = np.where(finite, d, 1e4).astype(np.float32)
        tmp = cv2.medianBlur(tmp, 5)
        d = np.where(tmp >= 1e4 * 0.99, np.inf, tmp).astype(np.float32)
    d[d > max_range] = np.inf
    if ground_reject and altitude > 0.3:
        d[d >= ground_distance(d.shape, altitude, hfov_deg, pitch_rad) * 0.88] = np.inf
    return d


def ground_distance(shape, altitude, hfov_deg=90.0, pitch_rad=0.0):
    """Euclidean distance at which each pixel's ray meets flat ground.
    Ray in camera frame d = (f, du, dv) with dv positive downward; pitch is
    AirSim's (nose-up positive).  Pixels looking at/above the horizon -> inf."""
    h, w = shape
    f = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    dv = (np.arange(h, dtype=np.float32) - (h - 1) / 2.0)[:, None]
    du = (np.arange(w, dtype=np.float32) - (w - 1) / 2.0)[None, :]
    down = dv * math.cos(pitch_rad) - f * math.sin(pitch_rad)
    norm = np.sqrt(f * f + du * du + dv * dv)
    with np.errstate(divide='ignore', invalid='ignore'):
        g = np.where(down > 1e-3, altitude * norm / down, np.inf)
    return g.astype(np.float32)


def zone_label(obs):
    return ''.join(k[0].upper() if obs[k] else '-' for k in ('left', 'center', 'right'))


# ---------------------------------------------------------------- planner
class AvoidancePlanner:
    """Rule-based rerouting from the report, as a timed state machine.

    Output of `step()` is a body-frame command: (vx_fwd, vy_right, yaw_rate_deg,
    label, event) where event is None or a (type, severity, text) tuple.
    """

    def __init__(self):
        self.maneuver = None          # dict(until, vx, vy, yaw, label, phase)
        self.last_label = 'IDLE'
        self.spin_dir = 0             # remembered rotation direction (+1 right / -1 left)
        self.spin_until = 0.0

    def reset(self):
        self.maneuver = None
        self.spin_dir = 0

    def step(self, obs, cfg, heading_rate=0.0, now=None):
        now = time.time() if now is None else now
        fwd = cfg['forward_speed']
        side = cfg['avoid_speed_sideways']
        hard_stop = cfg['hard_stop']
        dc = obs['min_dist_center']
        event = None

        # SAFETY (addition): something inside the hard-stop distance -> back away first
        if dc < hard_stop and not (self.maneuver and self.maneuver['phase'] == 'backoff'
                                   and now < self.maneuver['until']):
            self.maneuver = dict(until=now + 1.0, vx=-0.6, vy=0.0, yaw=0.0,
                                 label='TOO CLOSE → BACKING OFF', phase='backoff')
            event = ('Obstacle too close', 'HIGH', f'Centre obstacle at {fmt(dc)} m — backing off')
            return -0.6, 0.0, 0.0, self.maneuver['label'], event

        # an avoidance manoeuvre is in progress: keep executing it
        if self.maneuver and now < self.maneuver['until']:
            m = self.maneuver
            # abort a forward-bias step as soon as the centre is blocked again
            if m['phase'] == 'bias' and obs['center']:
                self.maneuver = None
            else:
                return m['vx'], m['vy'], m['yaw'], m['label'], None
        if self.maneuver and self.maneuver['phase'] == 'avoid' and not obs['center']:
            # Section 5.2.6 forward-progress bias: after a sidestep, take a short
            # forward step so the drone does not oscillate sideways forever.
            # FIX: only when the centre is actually clear (beyond the look-ahead
            # threshold) — previously it only checked hard_stop, which pushed the
            # drone back into a wall that was still 2-5 m ahead.
            self.maneuver = dict(until=now + cfg['bias_duration'], vx=fwd * 0.6, vy=0.0,
                                 yaw=0.0, label='FORWARD BIAS STEP (anti-stall)', phase='bias')
            return 0.6 * fwd, 0.0, 0.0, self.maneuver['label'], None
        self.maneuver = None

        if obs['center']:
            # Section 3.6 / 5.2.4 priority: centre blocked -> clearer side -> rotate
            vx, vy, yaw = 0.0, 0.0, 0.0
            left_ok, right_ok = not obs['left'], not obs['right']
            if left_ok and right_ok:
                # both clear (report picks left); choose the side with more room
                go_left = obs['min_dist_left'] >= obs['min_dist_right']
            else:
                go_left = left_ok
            if left_ok or right_ok:
                vy = -side if go_left else side
                label = f"CENTRE BLOCKED → SHIFT {'LEFT' if go_left else 'RIGHT'}"
                sev = 'LOW'
            else:
                # all blocked: rotate toward the side with more room, and keep
                # that direction for a few seconds so it does not dither L/R
                if not self.spin_dir or now > self.spin_until:
                    self.spin_dir = -1 if obs['min_dist_left'] > obs['min_dist_right'] else 1
                self.spin_until = now + 4.0
                yaw = 30.0 * self.spin_dir
                label = f"ALL SECTORS BLOCKED → ROTATE {'LEFT' if self.spin_dir < 0 else 'RIGHT'}"
                sev = 'MEDIUM'
            self.maneuver = dict(until=now + cfg['avoid_duration'], vx=vx, vy=vy, yaw=yaw,
                                 label=label, phase='avoid')
            event = ('Obstacle avoidance manoeuvre', sev,
                     f"{label}. Distances L/C/R = {fmt(obs['min_dist_left'])} / "
                     f"{fmt(dc)} / {fmt(obs['min_dist_right'])} m")
            self.last_label = label
            return vx, vy, yaw, label, event

        vx, vy = fwd, 0.0
        label = 'PATH CLEAR → FORWARD'
        if obs['left'] and not obs['right']:
            vy = 0.5
            label = 'SLIGHT OBSTACLE LEFT → ADJUST RIGHT'
        elif obs['right'] and not obs['left']:
            vy = -0.5
            label = 'SLIGHT OBSTACLE RIGHT → ADJUST LEFT'
        self.last_label = label
        return vx, vy, heading_rate, label, event


def fmt(v):
    return '∞' if not np.isfinite(v) else f'{v:.1f}'
