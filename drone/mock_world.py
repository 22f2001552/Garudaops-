"""Built-in mock simulator used when AirSim is not running.

A small 2.5-D world (buildings, trees, poles, walking people, moving vehicles)
rendered by vectorised ray casting from a level forward camera.  It produces the
same things the AirSim backend produces — an RGB scene, a DepthPerspective image
in metres, and a per-pixel object-id map — plus a ground-truth temperature map
for the thermal view.  It is a test harness, not a physics simulator: it exists
so the dashboard, controller and avoidance logic can be exercised anywhere.
"""
import math
import time

import numpy as np

KIND_GROUND, KIND_SKY, KIND_BUILDING, KIND_TREE, KIND_POLE, KIND_PERSON, KIND_VEHICLE, KIND_WALL = range(8)
KIND_NAMES = {KIND_BUILDING: 'building', KIND_TREE: 'tree', KIND_POLE: 'pole',
              KIND_PERSON: 'person', KIND_VEHICLE: 'car', KIND_WALL: 'wall'}


class MockWorld:
    def __init__(self, width=480, height=270, hfov_deg=90.0, seed=7):
        self.W, self.H, self.hfov = width, height, hfov_deg
        self.f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        du = np.arange(width, dtype=np.float32) - (width - 1) / 2.0
        dv = np.arange(height, dtype=np.float32) - (height - 1) / 2.0
        self.du, self.dv = du, dv
        self.col_off = np.arctan2(du, self.f)                    # bearing offset per column
        self.hn = np.sqrt(self.f ** 2 + du ** 2)                 # horizontal ray norm per column
        self.norm = np.sqrt(self.hn[None, :] ** 2 + dv[:, None] ** 2)   # full ray norm (H,W)
        self._build()

    # ------------------------------------------------------------ world
    def _build(self):
        r = self.rng
        objs = []  # each: dict(kind, shape 'box'|'circle', geometry, height, color, temp)

        def box(x0, y0, x1, y1, h, kind=KIND_BUILDING, color=None, temp=31.0):
            objs.append(dict(kind=kind, shape='box', b=np.array([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)], np.float32),
                             h=h, color=np.array(color if color is not None else r.integers(95, 165, 3), np.float32), temp=temp))

        def circ(x, y, rad, h, kind, color, temp):
            objs.append(dict(kind=kind, shape='circle', c=np.array([x, y], np.float32), r=rad, h=h,
                             color=np.array(color, np.float32), temp=temp))

        # city blocks north of home (x = north, y = east)
        for bx in (40, 70, 100):
            for by in (-60, -30, 25, 55):
                if (bx, by) == (70, 25):
                    continue                    # open plaza (crowd gathering point)
                w, d = r.uniform(10, 16), r.uniform(10, 16)
                box(bx, by, bx + w, by + d, r.uniform(12, 30), temp=r.uniform(17, 23))
        # a wall + boxes straight ahead of take-off so avoidance triggers early
        box(16, -4, 18, 3, 9, KIND_WALL, (150, 150, 160), 19)
        box(26, 5, 30, 9, 8, KIND_WALL, (140, 120, 110), 19)
        box(-30, -30, -18, -14, 14, temp=20)
        box(-40, 20, -28, 34, 18, temp=21)
        # trees & poles
        for _ in range(55):
            x, y = r.uniform(-60, 130), r.uniform(-90, 90)
            if abs(x) < 8 and abs(y) < 8:
                continue
            circ(x, y, r.uniform(0.8, 1.8), r.uniform(6, 11), KIND_TREE, (40, r.uniform(110, 150), 50), 16.0)
        for i in range(16):
            circ(8 + i * 8, -12, 0.25, 8.0, KIND_POLE, (180, 180, 185), 21.0)
        self.static = objs

        # people: a crowd forming in the plaza + walkers
        self.plaza = np.array([77.0, 32.0])
        self.gather2 = np.array([12.0, 45.0])
        n = 70
        self.p_pos = np.column_stack([r.uniform(-20, 120, n), r.uniform(-70, 80, n)]).astype(np.float32)
        self.p_goal = np.zeros_like(self.p_pos)
        self.p_speed = r.uniform(0.6, 1.4, n).astype(np.float32)
        self.p_color = r.integers(60, 230, (n, 3)).astype(np.float32)
        self.p_role = np.where(np.arange(n) < 42, 0, np.where(np.arange(n) < 55, 1, 2))  # plaza / gather2 / walker
        for i in range(n):
            self.p_goal[i] = self._new_goal(i)
        # vehicles on two roads: y=-20 (north-south) and x=60 (east-west)
        self.v = []
        for i in range(8):
            ns = i < 5
            self.v.append(dict(ns=ns, s=r.uniform(-60, 130) if ns else r.uniform(-90, 90),
                               dirn=1 if i % 2 else -1, speed=r.uniform(4, 8),
                               color=np.array(r.integers(40, 230, 3), np.float32)))

    def _new_goal(self, i):
        r = self.rng
        if self.p_role[i] == 0:
            return self.plaza + r.normal(0, 4.5, 2)
        if self.p_role[i] == 1:
            return self.gather2 + r.normal(0, 3.0, 2)
        return np.array([r.uniform(-20, 120), r.uniform(-70, 80)])

    def step_agents(self, dt):
        self.t += dt
        delta = self.p_goal - self.p_pos
        dist = np.linalg.norm(delta, axis=1) + 1e-6
        arrived = dist < 1.0
        mill = (self.p_role < 2) & arrived
        step = (delta / dist[:, None]) * (self.p_speed * dt)[:, None]
        step[arrived] = 0
        self.p_pos += step.astype(np.float32)
        for i in np.where(arrived)[0]:
            if mill[i] and self.rng.random() > 0.02:
                self.p_pos[i] += self.rng.normal(0, 0.05, 2).astype(np.float32)   # milling about
                continue
            self.p_goal[i] = self._new_goal(i)
        for v in self.v:
            v['s'] += v['dirn'] * v['speed'] * dt
            lim = (-60, 130) if v['ns'] else (-90, 90)
            if v['s'] > lim[1]:
                v['s'] = lim[0]
            if v['s'] < lim[0]:
                v['s'] = lim[1]

    def dynamic_objects(self):
        objs = []
        for i, (x, y) in enumerate(self.p_pos):
            objs.append(dict(kind=KIND_PERSON, shape='circle', c=np.array([x, y], np.float32), r=0.32, h=1.75,
                             color=self.p_color[i], temp=34.5, pid=i))
        for v in self.v:
            if v['ns']:
                x, y = v['s'], -20.0 + (1.6 if v['dirn'] > 0 else -1.6)
                b = [x - 2.2, y - 0.9, x + 2.2, y + 0.9]
            else:
                x, y = 60.0 + (1.6 if v['dirn'] > 0 else -1.6), v['s']
                b = [x - 0.9, y - 2.2, x + 0.9, y + 2.2]
            objs.append(dict(kind=KIND_VEHICLE, shape='box', b=np.array(b, np.float32), h=1.5, color=v['color'], temp=41.0))
        return objs

    def all_objects(self):
        return self.static + self.dynamic_objects()

    # ------------------------------------------------------------ collision
    def collides(self, x, y, alt, radius=0.4):
        for o in self.static:
            if alt > o['h']:
                continue
            if o['shape'] == 'circle':
                if math.hypot(x - o['c'][0], y - o['c'][1]) < o['r'] + radius:
                    return KIND_NAMES[o['kind']]
            else:
                b = o['b']
                if b[0] - radius < x < b[2] + radius and b[1] - radius < y < b[3] + radius:
                    return KIND_NAMES[o['kind']]
        return None

    # ------------------------------------------------------------ render
    def render(self, x, y, alt, yaw, max_range=80.0):
        W, H = self.W, self.H
        ang = yaw + self.col_off
        dx, dy = np.cos(ang).astype(np.float32), np.sin(ang).astype(np.float32)
        dv = self.dv[:, None]
        hn = self.hn[None, :]
        # ground: horizontal distance where the ray reaches z=0
        with np.errstate(divide='ignore', invalid='ignore'):
            t_ground = np.where(dv > 0.5, alt * hn / np.maximum(dv, 1e-6), np.inf).astype(np.float32)
        t_best = np.broadcast_to(t_ground, (H, W)).copy()
        idmap = np.where(np.isfinite(t_best), 0, -1).astype(np.int32)    # 0 ground, -1 sky
        hmap = np.zeros((H, W), np.float32)                              # hit height above ground
        objs = self.all_objects()
        fwd = np.array([math.cos(yaw), math.sin(yaw)], np.float32)
        for k, o in enumerate(objs, start=1):
            if o['shape'] == 'circle':
                c = o['c'] - (x, y)
                dist = float(np.hypot(*c))
                if dist > max_range or (float(c @ fwd) < -o['r'] - 1):
                    continue
                b = dx * c[0] + dy * c[1]
                disc = o['r'] ** 2 - (dist ** 2 - b ** 2)
                with np.errstate(invalid='ignore'):
                    sq = np.sqrt(np.maximum(disc, 0))
                t = np.where(disc >= 0, b - sq, np.inf)
                t = np.where((t < 0.05) & (disc >= 0) & (b + sq > 0), 0.05, t)
                t = np.where(t > 0, t, np.inf)
            else:
                bx0, by0, bx1, by1 = o['b'] - (x, y, x, y)
                cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
                if math.hypot(cx, cy) > max_range + 20 or (cx * fwd[0] + cy * fwd[1] < -30):
                    continue
                with np.errstate(divide='ignore', invalid='ignore'):
                    idx = np.where(np.abs(dx) < 1e-6, 1e-6, dx)
                    idy = np.where(np.abs(dy) < 1e-6, 1e-6, dy)
                    tx1, tx2 = bx0 / idx, bx1 / idx
                    ty1, ty2 = by0 / idy, by1 / idy
                tnear = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
                tfar = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
                hit = tfar >= np.maximum(tnear, 0)
                t = np.where(hit, np.where(tnear > 0.05, tnear, 0.05), np.inf)
            if not np.isfinite(t).any():
                continue
            cols = np.isfinite(t)
            tc = t[None, cols]
            hray = alt - dv * tc / hn[:, cols]
            sub = (hray >= 0) & (hray <= o['h']) & (tc < t_best[:, cols])
            if not sub.any():
                continue
            tb = t_best[:, cols]
            tb[sub] = np.broadcast_to(tc, sub.shape)[sub]
            t_best[:, cols] = tb
            im = idmap[:, cols]
            im[sub] = k
            idmap[:, cols] = im
            hm = hmap[:, cols]
            hm[sub] = hray[sub]
            hmap[:, cols] = hm

        depth = (t_best * self.norm / hn).astype(np.float32)      # Euclidean (DepthPerspective)
        depth[idmap < 0] = np.inf
        scene, temp = self._shade(x, y, alt, dx, dy, t_best, idmap, hmap, depth, objs)
        return dict(scene=scene, depth=depth, idmap=idmap, objects=objs, thermal_c=temp)

    def _shade(self, x, y, alt, dx, dy, t_best, idmap, hmap, depth, objs):
        H, W = idmap.shape
        if not hasattr(self, '_sky'):
            rows = np.linspace(0, 1, H, dtype=np.float32)[:, None]
            self._sky = np.stack([235 - 60 * rows, 200 - 50 * rows, 150 - 40 * rows], -1) * np.ones((1, W, 1), np.float32)
            self._noise = self.rng.normal(0, 3, (H * 2, W * 2)).astype(np.float32)
        sky = self._sky
        n = len(objs)
        # lookup tables indexed by idmap+1  (0 -> sky, 1 -> ground, k+1 -> object k)
        ctab = np.zeros((n + 2, 3), np.float32)
        ttab = np.zeros(n + 2, np.float32)
        ktab = np.zeros(n + 2, np.int16)
        ttab[0] = 4.0
        ktab[0], ktab[1] = KIND_SKY, KIND_GROUND
        for k, o in enumerate(objs, start=2):
            ctab[k] = o['color']
            ttab[k] = o['temp']
            ktab[k] = o['kind']
        li = idmap + 1
        img = ctab[li]
        temp = ttab[li]
        kind = ktab[li]
        sk = kind == KIND_SKY
        img[sk] = sky[sk]
        g = kind == KIND_GROUND
        if g.any():
            tb = t_best[g]
            gx = x + tb * np.broadcast_to(dx[None, :], (H, W))[g]
            gy = y + tb * np.broadcast_to(dy[None, :], (H, W))[g]
            tile = ((np.floor(gx / 4) + np.floor(gy / 4)) % 2)[:, None] * 10
            col = np.array([70, 140, 95], np.float32) + tile
            road = (np.abs(gy + 20) < 3.2) | (np.abs(gx - 60) < 3.2)
            plaza = (np.abs(gx - 77) < 11) & (np.abs(gy - 32) < 11)
            col[road] = np.array([85, 85, 88], np.float32) + tile[road] * 0.4
            col[plaza] = np.array([150, 160, 170], np.float32) + tile[plaza] * 0.5
            stripe = road & ((np.abs(gy + 20) < 0.15) | (np.abs(gx - 60) < 0.15)) & ((np.floor(gx + gy) % 3) < 1.5)
            col[stripe] = (200, 230, 235)
            img[g] = col
            gt = 22 + tile[:, 0] * 0.15
            gt[road] = 29.5           # sun-heated asphalt
            gt[plaza] = 27.0
            temp[g] = gt
        per = kind == KIND_PERSON
        if per.any():
            head = per & (hmap > 1.45)
            legs = per & (hmap < 0.8)
            img[legs] = (50, 50, 60)
            img[head] = (120, 160, 210)
            temp[per] = 31.5
            temp[head] = 35.8
        veh = kind == KIND_VEHICLE
        if veh.any():
            top = veh & (hmap > 1.0)
            img[top] = img[top] * 0.55 + 40
            temp[veh] = 33.0
            temp[veh & (hmap < 0.7)] = 44.0        # engine / exhaust
        bld = kind == KIND_BUILDING
        if bld.any():
            hm = hmap % 3.2
            win = bld & (hm > 1.6) & (hm < 2.6)
            temp[bld] -= 0.08 * hmap[bld]
            img[win] = img[win] * 0.45 + (60, 40, 20)
            temp[win] += 2.5
        veg = (kind == KIND_TREE) | (kind == KIND_POLE)
        if veg.any():
            img[veg] *= 0.85 + 0.15 * np.clip(hmap[veg] / 10.0, 0, 1)[:, None]
        # distance fog
        fog = np.where(np.isfinite(depth), np.clip(depth / 110.0, 0, 0.65), 0).astype(np.float32)[..., None]
        img = img * (1 - fog) + sky * fog
        oy, ox = self.rng.integers(0, H), self.rng.integers(0, W)
        nz = self._noise[oy:oy + H, ox:ox + W]
        img = np.clip(img + nz[..., None], 0, 255).astype(np.uint8)
        temp = temp + nz * 0.08
        return img, temp


class MockDrone:
    """First-order multirotor model in NED-style local frame (x north, y east,
    z down) driven by the same velocity commands the AirSim backend uses."""

    def __init__(self, world):
        self.world = world
        self.reset()

    def reset(self):
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.cmd = dict(vx=0.0, vy=0.0, z=0.0, yaw_rate=0.0, until=0.0)
        self.armed = False
        self.landing = False
        self.collided = None
        self.last = time.time()

    def update(self):
        now = time.time()
        dt = min(0.2, now - self.last)
        self.last = now
        self.world.step_agents(dt)
        c = self.cmd
        active = now < c['until']
        tvx, tvy = (c['vx'], c['vy']) if active else (0.0, 0.0)
        tz = c['z']
        if self.landing:
            tz = 0.0
            tvx = tvy = 0.0
        if not self.armed:
            tvx = tvy = 0.0
            tz = 0.0
        if self.collided:
            tvx = tvy = 0.0
        tau = 0.35
        a = min(1.0, dt / tau)
        self.vel[0] += (tvx - self.vel[0]) * a
        self.vel[1] += (tvy - self.vel[1]) * a
        vz = np.clip((tz - self.pos[2]) * 1.2, -3.0, 2.0)
        self.vel[2] += (vz - self.vel[2]) * a
        if active and self.armed:
            self.yaw = (self.yaw + math.radians(c['yaw_rate']) * dt + math.pi) % (2 * math.pi) - math.pi
        new = self.pos + self.vel * dt
        new[2] = min(new[2], 0.0)
        hit = self.world.collides(new[0], new[1], -new[2])
        if hit and -new[2] > 0.3:
            if not self.collided:
                self.collided = hit
            self.vel[:2] = 0
            new[:2] = self.pos[:2]
        self.pos = new
        if self.landing and -self.pos[2] < 0.05:
            self.landing = False
            self.armed = False          # touched down -> disarm
