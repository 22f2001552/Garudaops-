"""Sensor visualisations streamed to the dashboard.

  detection_view  – RGB + detection boxes, depth-obstacle boxes, L/C/R sectors, decision arrow
  depth_view      – DepthPerspective heat map (near = red), threshold contours, sector minima
  crowd_view      – person-density heat map (Gaussian kernel at each person's foot point)
  thermal_view    – thermal palette (mock: simulated temperatures; AirSim: Infrared image if
                    enabled, otherwise a pseudo-thermal image synthesised from RGB+depth+detections)
  edges_view      – Canny edges (the report's RGB pipeline, Section 3.4)
"""
import math

import cv2
import numpy as np

S = 2  # render overlays at 2x the camera resolution so text stays crisp
FONT = cv2.FONT_HERSHEY_SIMPLEX
CYAN, AMBER, RED, GREEN, WHITE, VIOLET = (209, 216, 66), (90, 180, 245), (80, 80, 240), (150, 220, 90), (235, 240, 240), (230, 140, 190)
COLORS = {'person': CYAN, 'car': AMBER, 'bicycle': AMBER, 'object': VIOLET}


def up(img):
    return cv2.resize(img, None, fx=S, fy=S, interpolation=cv2.INTER_LINEAR)


def chip(img, text, org, fg=WHITE, bg=(20, 28, 36), scale=0.5, th=1, alpha=0.75):
    (tw, tht), base = cv2.getTextSize(text, FONT, scale, th)
    x, y = int(org[0]), int(org[1])
    x0, y0, x1, y1 = x, y - tht - 6, x + tw + 10, y + base + 2
    x0, y0 = max(0, x0), max(0, y0)
    roi = img[y0:y1, x0:x1]
    if roi.size:
        img[y0:y1, x0:x1] = (roi * (1 - alpha) + np.array(bg) * alpha).astype(np.uint8)
    cv2.putText(img, text, (x + 5, y - 2), FONT, scale, fg, th, cv2.LINE_AA)


def title(img, text, sub=None):
    chip(img, text, (10, 28), fg=(220, 240, 120), scale=0.55)
    if sub:
        chip(img, sub, (10, 54), fg=(190, 205, 215), scale=0.42)


def bracket_box(img, b, color, t=2):
    x0, y0, x1, y1 = (int(v) for v in b)
    L = max(6, int(min(x1 - x0, y1 - y0) * 0.25))
    for (px, py, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(img, (px, py), (px + dx * L, py), color, t, cv2.LINE_AA)
        cv2.line(img, (px, py), (px, py + dy * L), color, t, cv2.LINE_AA)
    cv2.rectangle(img, (x0, y0), (x1, y1), color, 1, cv2.LINE_AA)


def dist_txt(d):
    return '--' if d is None or not np.isfinite(d) else f'{d:.1f}m'


# ------------------------------------------------------------------ analysis
def obstacle_boxes(depth_proc, near=8.0, roi_start=0):
    """Connected regions of the cleaned depth image closer than `near` metres."""
    m = (depth_proc < near).astype(np.uint8)
    if roi_start:
        m[:roi_start // 2] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = []
    area_min = depth_proc.size * 0.004
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a < area_min:
            continue
        reg = depth_proc[y:y + h, x:x + w][lab[y:y + h, x:x + w] == i]
        out.append(dict(box=(x, y, x + w, y + h), dist=float(np.min(reg))))
    out.sort(key=lambda o: o['dist'])
    return out[:8]


def geolocate(dets, depth_raw, st, hfov_deg):
    """Estimate each detection's distance (median depth in the box) and its
    ground position in the local frame, from drone pose + pixel bearing."""
    h, w = depth_raw.shape
    f = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    alt = max(0.0, -st['z'])
    for d in dets:
        x0, y0, x1, y1 = d['box']
        xi0, xi1 = int(max(0, x0 + (x1 - x0) * 0.25)), int(min(w, x1 - (x1 - x0) * 0.25 + 1))
        yi0, yi1 = int(max(0, y0 + (y1 - y0) * 0.2)), int(min(h, y1 - (y1 - y0) * 0.1 + 1))
        patch = depth_raw[yi0:yi1, xi0:xi1]
        patch = patch[np.isfinite(patch)]
        if patch.size:
            dist = float(np.median(patch))
        else:
            ref = 1.7 if d['label'] == 'person' else 1.5
            dist = f * ref / max(2.0, (y1 - y0))
        d['dist'] = round(dist, 1)
        bearing = st['yaw'] + math.atan2(((x0 + x1) / 2) - (w - 1) / 2, f)
        rng = math.sqrt(max(0.0, dist * dist - max(0.0, alt - 0.8) ** 2))
        d['wx'] = round(st['x'] + rng * math.cos(bearing), 2)
        d['wy'] = round(st['y'] + rng * math.sin(bearing), 2)
    return dets


def crowd_level(n, cfg):
    if n >= cfg['crowd_very_high']:
        return 'VERY HIGH'
    if n >= cfg['crowd_high']:
        return 'HIGH'
    if n >= cfg['crowd_moderate']:
        return 'MODERATE'
    return 'LOW'


# ------------------------------------------------------------------ views
def detection_view(scene, dets, obst, obs, cfg, decision, roi_start):
    img = up(scene)
    H, W = img.shape[:2]
    # ROI + sector guides (detect_obstacle_regions geometry)
    ry = roi_start * S
    for x in range(0, W, 14):
        cv2.line(img, (x, ry), (x + 7, ry), (200, 200, 200), 1)
    for k in (1, 2):
        cv2.line(img, (W * k // 3, ry), (W * k // 3, H), (200, 200, 200), 1, cv2.LINE_AA)
    blocked = [obs['left'], obs['center'], obs['right']]
    ov = img.copy()
    for k, b in enumerate(blocked):
        if b:
            cv2.rectangle(ov, (W * k // 3, ry), (W * (k + 1) // 3, H), RED, -1)
    img = cv2.addWeighted(ov, 0.18, img, 0.82, 0)
    for o in obst:
        x0, y0, x1, y1 = (v * S for v in o['box'])
        col = RED if o['dist'] < cfg['look_ahead_threshold'] else AMBER
        cv2.rectangle(img, (x0, y0), (x1, y1), col, 1, cv2.LINE_AA)
        chip(img, f"OBSTACLE {o['dist']:.1f}m", (x0, max(16, y0 - 2)), fg=col, scale=0.42)
    for d in dets:
        col = COLORS.get(d['label'], VIOLET)
        b = [v * S for v in d['box']]
        bracket_box(img, b, col)
        if (b[3] - b[1]) > 40:
            chip(img, f"{d['label'].upper()} {d['conf']:.2f} {dist_txt(d.get('dist'))}",
                 (b[0], max(16, b[1] - 3)), fg=col, scale=0.38)
    # crosshair
    cx, cy = W // 2, H // 2
    cv2.circle(img, (cx, cy), 16, WHITE, 1, cv2.LINE_AA)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cv2.line(img, (cx + dx * 22, cy + dy * 22), (cx + dx * 34, cy + dy * 34), WHITE, 1, cv2.LINE_AA)
    # sector readout
    for k, key in enumerate(('left', 'center', 'right')):
        d = obs['min_dist_' + key]
        col = RED if obs[key] else GREEN
        chip(img, f"{key[0].upper()} {dist_txt(d)}", (W * k // 3 + W // 6 - 38, H - 14), fg=col, scale=0.55, th=2)
    # decision arrow
    _arrow(img, decision, (W // 2, int(H * 0.30)))
    n_p = sum(1 for d in dets if d['label'] == 'person')
    n_v = sum(1 for d in dets if d['label'] in ('car', 'bus', 'truck', 'motorcycle'))
    title(img, 'OBJECT DETECTION OVERLAY', f"PERSONS {n_p}  VEHICLES {n_v}  OBSTACLES {len(obst)}")
    return img


def _arrow(img, decision, c):
    label = (decision or '').upper()
    x, y = c
    col = RED if ('BLOCKED' in label or 'BACKING' in label) else (AMBER if 'ADJUST' in label or 'BIAS' in label else GREEN)
    if 'BACKING' in label:
        cv2.arrowedLine(img, (x, y - 30), (x, y + 30), col, 4, cv2.LINE_AA, tipLength=0.35)
    elif 'ROTATE' in label and 'LEFT' in label:
        cv2.ellipse(img, (x, y), (30, 30), 0, -70, 220, col, 3, cv2.LINE_AA)
        cv2.arrowedLine(img, (x - 22, y - 22), (x - 30, y - 4), col, 3, cv2.LINE_AA, tipLength=0.9)
    elif 'ROTATE' in label:
        cv2.ellipse(img, (x, y), (30, 30), 0, -40, 250, col, 3, cv2.LINE_AA)
        cv2.arrowedLine(img, (x + 22, y - 22), (x + 30, y - 4), col, 3, cv2.LINE_AA, tipLength=0.9)
    elif 'LEFT' in label:
        cv2.arrowedLine(img, (x + 40, y), (x - 40, y), col, 4, cv2.LINE_AA, tipLength=0.35)
    elif 'RIGHT' in label:
        cv2.arrowedLine(img, (x - 40, y), (x + 40, y), col, 4, cv2.LINE_AA, tipLength=0.35)
    elif label and 'HOLD' not in label and 'IDLE' not in label and 'LAND' not in label:
        cv2.arrowedLine(img, (x, y + 30), (x, y - 30), col, 4, cv2.LINE_AA, tipLength=0.35)
    if label:
        (tw, _), _ = cv2.getTextSize(label, FONT, 0.5, 1)
        chip(img, label, (x - tw // 2 - 5, y + 62), fg=col, scale=0.5)


def depth_view(depth_raw, depth_proc, obs, cfg, roi_start, max_range=30.0):
    d = np.where(np.isfinite(depth_raw), depth_raw, max_range)
    d = np.clip(d, 0, max_range)
    norm = (255 * (1.0 - d / max_range)).astype(np.uint8)      # near -> high -> red
    heat = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    heat[~np.isfinite(depth_raw)] = (40, 22, 14)
    img = up(heat)
    H, W = img.shape[:2]
    # threshold contour on the cleaned depth (what the planner actually sees)
    m = (depth_proc < cfg['look_ahead_threshold']).astype(np.uint8)
    cnts, _ = cv2.findContours(up(m * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cnts, -1, WHITE, 2, cv2.LINE_AA)
    ry = roi_start * S
    cv2.line(img, (0, ry), (W, ry), WHITE, 1, cv2.LINE_AA)
    chip(img, f'ROI {int(cfg["roi_height_ratio"] * 100)}%', (W - 90, ry - 4), scale=0.4)
    for k, key in enumerate(('left', 'center', 'right')):
        x0, x1 = W * k // 3, W * (k + 1) // 3
        col = RED if obs[key] else GREEN
        cv2.rectangle(img, (x0 + 2, ry + 2), (x1 - 2, H - 2), col, 2 if obs[key] else 1)
        chip(img, f"{key.upper()} min {dist_txt(obs['min_dist_' + key])}", (x0 + 8, H - 12), fg=col, scale=0.45)
    # legend
    lg = cv2.applyColorMap(np.linspace(255, 0, 160).astype(np.uint8)[None, :].repeat(10, 0), cv2.COLORMAP_TURBO)
    img[60:70, W - 180:W - 20] = lg
    chip(img, '0m', (W - 190, 88), scale=0.38)
    chip(img, f'{int(max_range)}m+', (W - 52, 88), scale=0.38)
    title(img, 'DEPTH HEATMAP', f"DepthPerspective · threshold {cfg['look_ahead_threshold']:.1f} m (white contour)")
    return img


def crowd_view(scene, dets, cfg, level):
    h, w = scene.shape[:2]
    q = 4
    dens = np.zeros((h // q, w // q), np.float32)
    persons = [d for d in dets if d['label'] == 'person']
    for d in persons:
        x0, y0, x1, y1 = d['box']
        fx, fy = int(((x0 + x1) / 2) / q), int(min(h - 1, y1) / q)
        if 0 <= fx < dens.shape[1] and 0 <= fy < dens.shape[0]:
            dens[fy, fx] += 1.0
    if persons:
        hs = np.median([d['box'][3] - d['box'][1] for d in persons])
        sigma = max(1.5, hs * 1.4 / q)
        dens = cv2.GaussianBlur(dens, (0, 0), sigma) * (2 * math.pi * sigma * sigma)
    dens = cv2.resize(dens, (w * S, h * S), interpolation=cv2.INTER_CUBIC)
    sat = cfg.get('crowd_heat_saturation', 4.0)
    n = np.clip(dens / sat, 0, 1)
    gray = cv2.cvtColor(cv2.cvtColor(up(scene), cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
    base = (gray * 0.55).astype(np.uint8)
    heat = cv2.applyColorMap((n * 255).astype(np.uint8), cv2.COLORMAP_JET)
    a = np.clip(n * 1.6, 0, 0.85)[..., None]
    img = (base * (1 - a) + heat * a).astype(np.uint8)
    H, W = img.shape[:2]
    for d in persons:
        x0, y0, x1, y1 = (v * S for v in d['box'])
        cv2.circle(img, (int((x0 + x1) / 2), int(y1)), 3, WHITE, -1, cv2.LINE_AA)
    if persons and n.max() > 0.05:
        yy, xx = np.unravel_index(np.argmax(n), n.shape)
        cv2.circle(img, (int(xx), int(yy)), 36, (255, 255, 255), 2, cv2.LINE_AA)
        chip(img, 'DENSITY HOTSPOT', (int(xx) + 40, int(yy)), scale=0.42)
    lv_col = {'LOW': GREEN, 'MODERATE': (120, 220, 240), 'HIGH': AMBER, 'VERY HIGH': RED}[level]
    title(img, 'CROWD-DENSITY HEATMAP', f'PERSONS IN VIEW {len(persons)} · LEVEL {level}')
    chip(img, f'CROWD {level}', (W - 190, H - 14), fg=lv_col, scale=0.6, th=2)
    if not persons:
        chip(img, 'NO PERSONS DETECTED IN FRAME', (W // 2 - 150, H // 2), scale=0.55)
    return img


def synth_thermal(scene, depth_raw, dets, infrared=None):
    """Temperatures in deg C.  Uses AirSim's Infrared image when given; otherwise a
    pseudo-thermal estimate (sunlit surfaces warmer, sky cold, people/vehicles hot)."""
    h, w = scene.shape[:2]
    if infrared is not None:
        ir = cv2.resize(infrared, (w, h)).astype(np.float32)
        return 5.0 + ir / 255.0 * 40.0, 'AIRSIM INFRARED'
    gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    gray = cv2.GaussianBlur(gray, (0, 0), 2)
    t = 20.0 + 12.0 * gray
    t[~np.isfinite(depth_raw)] = 4.0
    blob = np.zeros((h, w), np.float32)
    for d in dets:
        x0, y0, x1, y1 = (int(v) for v in d['box'])
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        ax, ay = max(1, (x1 - x0) // 2), max(1, (y1 - y0) // 2)
        if d['label'] == 'person':
            cv2.ellipse(blob, (cx, cy), (ax, ay), 0, 0, 360, 15.0, -1)
            cv2.ellipse(blob, (cx, y0 + ay // 3), (max(1, ax // 2), max(1, ay // 4)), 0, 0, 360, 18.0, -1)
        elif d['label'] in ('car', 'bus', 'truck', 'motorcycle'):
            cv2.rectangle(blob, (x0, cy), (x1, y1), 22.0, -1)
            cv2.rectangle(blob, (x0, y0), (x1, cy), 10.0, -1)
    blob = cv2.GaussianBlur(blob, (0, 0), 1.5)
    t = np.maximum(t, 20.0 + blob)
    return t, 'PSEUDO-THERMAL (RGB+DEPTH+DETECTIONS)'


def thermal_view(temp_c, src_label, cfg):
    lo, hi = cfg.get('thermal_min_c', 0.0), cfg.get('thermal_max_c', 45.0)
    n = np.clip((temp_c - lo) / (hi - lo), 0, 1)
    img = up(cv2.applyColorMap((n * 255).astype(np.uint8), cv2.COLORMAP_INFERNO))
    H, W = img.shape[:2]
    hot = (cv2.GaussianBlur(temp_c, (0, 0), 1) > cfg.get('heat_alert_c', 34.0)).astype(np.uint8)
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(hot)
    sigs = 0
    for i in range(1, nlab):
        x, y, w, h, a = stats[i]
        if a < 6:
            continue
        sigs += 1
        tmax = float(temp_c[y:y + h, x:x + w].max())
        cv2.rectangle(img, (x * S - 3, y * S - 3), ((x + w) * S + 3, (y + h) * S + 3), (120, 255, 255), 1, cv2.LINE_AA)
        if h * S > 14:
            chip(img, f'{tmax:.1f}C', (x * S, y * S - 5), fg=(120, 255, 255), scale=0.36)
    yy, xx = np.unravel_index(np.argmax(temp_c), temp_c.shape)
    cv2.drawMarker(img, (int(xx * S), int(yy * S)), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
    cx, cy = W // 2, H // 2
    cv2.drawMarker(img, (cx, cy), WHITE, cv2.MARKER_CROSS, 26, 1)
    spot = float(temp_c[temp_c.shape[0] // 2, temp_c.shape[1] // 2])
    chip(img, f'SPOT {spot:.1f}C', (cx + 18, cy - 10), scale=0.45)
    bar = cv2.applyColorMap(np.linspace(255, 0, 180).astype(np.uint8)[:, None].repeat(12, 1), cv2.COLORMAP_INFERNO)
    img[70:250, W - 34:W - 22] = bar
    chip(img, f'{hi:.0f}C', (W - 82, 80), scale=0.38)
    chip(img, f'{lo:.0f}C', (W - 76, 252), scale=0.38)
    title(img, 'THERMAL VISUALIZATION', f'{src_label} · MAX {temp_c.max():.1f}C · HEAT SIGNATURES {sigs}')
    return img


def edges_view(scene, roi_start, lo=100, hi=200):
    gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    e = cv2.Canny(gray, int(lo), int(hi))              # report used 100/200 (Sec. 3.3)
    img = np.zeros((*e.shape, 3), np.uint8)
    img[:] = (30, 20, 12)
    img[e > 0] = (230, 230, 90)
    img = up(img)
    cv2.line(img, (0, roi_start * S), (img.shape[1], roi_start * S), (120, 120, 120), 1)
    title(img, f'EDGE DETECTION (CANNY {int(lo)}/{int(hi)})', 'RGB pipeline from report §3.4 — reference only, depth drives decisions')
    return img


def quad_view(views):
    tiles = []
    for k in ('detection', 'depth', 'crowd', 'thermal'):
        v = views.get(k)
        if v is None:
            v = np.zeros((270 * S, 480 * S, 3), np.uint8)
        tiles.append(cv2.resize(v, (480, 270), interpolation=cv2.INTER_AREA))
    top = np.hstack(tiles[:2])
    bot = np.hstack(tiles[2:])
    q = np.vstack([top, bot])
    cv2.line(q, (480, 0), (480, 540), (60, 60, 60), 2)
    cv2.line(q, (0, 270), (960, 270), (60, 60, 60), 2)
    return q


# ------------------------------------------------------------------ evidence timestamp
def fmt_ts(dt):
    """2026-11-27 00:12:34.567 UTC+05:30  (local time, millisecond precision)."""
    z = dt.strftime('%z') or '+0000'
    return f"{dt:%Y-%m-%d %H:%M:%S}.{dt.microsecond // 1000:03d} UTC{z[:3]}:{z[3:]}"


def fmt_elapsed(sec):
    h, rem = divmod(max(0.0, sec), 3600)
    m, s = divmod(rem, 60)
    return f'{int(h):02d}:{int(m):02d}:{s:06.3f}'


def stamp(img, dt, frame_no, source, rec_elapsed=None):
    """Append a CCTV-style evidence strip under the image (never covers the picture):
    capture date/time with ms · recording clock · frame number · source."""
    H, W = img.shape[:2]
    bar = np.zeros((34, W, 3), np.uint8)
    bar[:] = (14, 14, 14)
    cv2.line(bar, (0, 0), (W, 0), (70, 70, 70), 1)
    cv2.putText(bar, fmt_ts(dt), (10, 23), FONT, 0.58, WHITE, 1, cv2.LINE_AA)
    if rec_elapsed is not None:
        mid = f'REC {fmt_elapsed(rec_elapsed)}'
        (tw, _), _ = cv2.getTextSize(mid, FONT, 0.58, 2)
        x = W // 2 - tw // 2
        cv2.circle(bar, (x - 14, 17), 6, (40, 40, 230), -1, cv2.LINE_AA)
        cv2.putText(bar, mid, (x, 23), FONT, 0.58, (90, 90, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(bar, 'LIVE', (W // 2 - 20, 23), FONT, 0.58, GREEN, 1, cv2.LINE_AA)
    right = f'FRAME {frame_no:06d}  {source}'
    (tw, _), _ = cv2.getTextSize(right, FONT, 0.5, 1)
    cv2.putText(bar, right, (W - tw - 10, 23), FONT, 0.5, (190, 200, 210), 1, cv2.LINE_AA)
    return np.vstack([img, bar])
