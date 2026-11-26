"""Drone backends with one interface, so the controller does not care whether
it is flying AirSim or the built-in mock.

All calls are made from the controller thread only (the AirSim RPC client is
not thread-safe).  Frame: local NED — x north, y east, z down (metres);
yaw in radians, clockwise from north.
"""
import math
import time

import numpy as np

HFOV_DEG = 90.0


class Backend:
    name = 'base'
    hfov_deg = HFOV_DEG

    def connect(self): ...
    def arm_takeoff(self): ...
    def land(self): ...
    def disarm(self): ...
    def hover(self): ...
    def move(self, vx, vy, z, yaw_rate_deg, duration=0.5): ...
    def state(self): ...
    def images(self): ...
    def detections(self, cam_w, cam_h): return []
    def reset(self): ...
    def close(self): ...


# ====================================================================== AirSim
class AirSimBackend(Backend):
    name = 'AIRSIM'

    def __init__(self, host='127.0.0.1', vehicle='', camera='0', detect_meshes=None, use_infrared=False):
        import airsim  # noqa: imported lazily so mock mode works without it
        self.airsim = airsim
        self.host, self.vehicle, self.camera = host, vehicle, camera
        self.detect_meshes = detect_meshes or []
        self.use_infrared = use_infrared
        self.client = None
        self.home_gps = None
        self.detection_api = False

    def connect(self):
        a = self.airsim
        self.client = a.MultirotorClient(ip=self.host, timeout_value=10)
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle)
        try:
            g = self.client.getMultirotorState(self.vehicle).gps_location
            self.home_gps = (g.latitude, g.longitude)
        except Exception:
            self.home_gps = None
        # AirSim >=1.5 detection API: boxes for meshes whose names match filters
        if self.detect_meshes:
            try:
                it = a.ImageType.Scene
                self.client.simClearDetectionMeshNames(self.camera, it, vehicle_name=self.vehicle)
                self.client.simSetDetectionFilterRadius(self.camera, it, 120 * 100, vehicle_name=self.vehicle)
                for m in self.detect_meshes:
                    self.client.simAddDetectionFilterMeshName(self.camera, it, m, vehicle_name=self.vehicle)
                self.detection_api = True
            except Exception as e:  # older AirSim builds
                print('[AirSim] detection API unavailable:', e)

    def arm_takeoff(self):
        self.client.enableApiControl(True, self.vehicle)
        self.client.armDisarm(True, self.vehicle)
        self.client.takeoffAsync(vehicle_name=self.vehicle)      # not joined: controller keeps streaming

    def land(self):
        self.client.landAsync(vehicle_name=self.vehicle)

    def disarm(self):
        self.client.armDisarm(False, self.vehicle)

    def hover(self):
        self.client.hoverAsync(vehicle_name=self.vehicle)

    def move(self, vx, vy, z, yaw_rate_deg, duration=0.5):
        a = self.airsim
        self.client.moveByVelocityZAsync(float(vx), float(vy), float(z), duration,
                                         drivetrain=a.DrivetrainType.MaxDegreeOfFreedom,
                                         yaw_mode=a.YawMode(True, float(yaw_rate_deg)),
                                         vehicle_name=self.vehicle)

    def state(self):
        a = self.airsim
        s = self.client.getMultirotorState(self.vehicle)
        k = s.kinematics_estimated
        pitch, roll, yaw = a.to_eularian_angles(k.orientation)
        col = self.client.simGetCollisionInfo(self.vehicle)
        g = s.gps_location
        return dict(x=k.position.x_val, y=k.position.y_val, z=k.position.z_val,
                    vx=k.linear_velocity.x_val, vy=k.linear_velocity.y_val, vz=k.linear_velocity.z_val,
                    yaw=yaw, pitch=pitch, roll=roll,
                    lat=g.latitude, lon=g.longitude,
                    landed=(s.landed_state == a.LandedState.Landed),
                    collided=col.object_name if col.has_collided else None)

    def images(self):
        a = self.airsim
        reqs = [a.ImageRequest(self.camera, a.ImageType.Scene, False, False),
                a.ImageRequest(self.camera, a.ImageType.DepthPerspective, True, False)]
        if self.use_infrared:
            reqs.append(a.ImageRequest(self.camera, a.ImageType.Infrared, False, False))
        res = self.client.simGetImages(reqs, vehicle_name=self.vehicle)
        if not res or len(res) < 2:
            return None
        r0, r1 = res[0], res[1]
        if r0.height == 0 or r1.height == 0 or r1.image_data_float is None:
            return None
        buf = np.frombuffer(r0.image_data_uint8, dtype=np.uint8)
        ch = buf.size // (r0.height * r0.width)
        scene = buf.reshape(r0.height, r0.width, ch)[:, :, :3].copy()
        # Same as get_depth_image_reliable(): zero -> inf
        depth = np.array(r1.image_data_float, dtype=np.float32).reshape(r1.height, r1.width)
        depth[depth == 0] = np.inf
        out = dict(scene=scene, depth=depth)
        if self.use_infrared and len(res) > 2 and res[2].height:
            b = np.frombuffer(res[2].image_data_uint8, dtype=np.uint8)
            c = b.size // (res[2].height * res[2].width)
            out['infrared'] = b.reshape(res[2].height, res[2].width, c)[:, :, 0].copy()
        return out

    def detections(self, cam_w, cam_h):
        if not self.detection_api:
            return []
        try:
            raw = self.client.simGetDetections(self.camera, self.airsim.ImageType.Scene, vehicle_name=self.vehicle)
        except Exception:
            return []
        out = []
        for d in raw or []:
            name = d.name.lower()
            label = ('person' if any(k in name for k in ('person', 'pedestrian', 'human', 'char')) else
                     'car' if any(k in name for k in ('car', 'vehicle', 'truck', 'suv', 'van')) else 'object')
            b = d.box2D
            out.append(dict(label=label, conf=1.0, box=(b.min.x_val, b.min.y_val, b.max.x_val, b.max.y_val),
                            source='AIRSIM-API', name=d.name))
        return out

    def reset(self):
        self.client.reset()
        self.client.enableApiControl(True, self.vehicle)

    def close(self):
        try:
            self.client.armDisarm(False, self.vehicle)
            self.client.enableApiControl(False, self.vehicle)
        except Exception:
            pass


# ====================================================================== mock
class MockBackend(Backend):
    name = 'MOCK SIM'

    def __init__(self, home=(17.4435, 78.3772)):
        from .mock_world import MockWorld, MockDrone
        self.world = MockWorld(hfov_deg=HFOV_DEG)
        self.drone = MockDrone(self.world)
        self.home_gps = home
        self._last = None

    def connect(self):
        time.sleep(0.2)

    def arm_takeoff(self):
        d = self.drone
        d.armed = True
        d.landing = False
        d.collided = None
        d.cmd.update(z=-3.0, vx=0, vy=0, yaw_rate=0, until=time.time() + 3)

    def land(self):
        self.drone.landing = True

    def disarm(self):
        self.drone.armed = False
        self.drone.cmd.update(z=0.0)

    def hover(self):
        self.drone.cmd.update(vx=0.0, vy=0.0, yaw_rate=0.0, until=time.time() + 0.5)

    def move(self, vx, vy, z, yaw_rate_deg, duration=0.5):
        self.drone.cmd.update(vx=float(vx), vy=float(vy), z=float(z), yaw_rate=float(yaw_rate_deg),
                              until=time.time() + duration)

    def state(self):
        d = self.drone
        d.update()
        lat0, lon0 = self.home_gps
        return dict(x=d.pos[0], y=d.pos[1], z=d.pos[2], vx=d.vel[0], vy=d.vel[1], vz=d.vel[2],
                    yaw=d.yaw, pitch=0.0, roll=0.0,
                    lat=lat0 + d.pos[0] / 111320.0,
                    lon=lon0 + d.pos[1] / (111320.0 * math.cos(math.radians(lat0))),
                    landed=(-d.pos[2] < 0.1 and not d.armed),
                    collided=d.collided)

    def images(self):
        d = self.drone
        r = self.world.render(d.pos[0], d.pos[1], max(0.3, -d.pos[2]), d.yaw)
        self._last = r
        return dict(scene=r['scene'], depth=r['depth'], thermal_c=r['thermal_c'])

    def detections(self, cam_w, cam_h):
        """Ground-truth boxes from the id map (same role as AirSim's detection API)."""
        from .mock_world import KIND_PERSON, KIND_VEHICLE
        r = self._last
        if r is None:
            return []
        idmap, objs = r['idmap'], r['objects']
        kinds = np.array([0] + [o['kind'] for o in objs])
        ids = idmap.ravel()
        valid = ids > 0
        sel = np.zeros_like(valid)
        sel[valid] = np.isin(kinds[ids[valid]], (KIND_PERSON, KIND_VEHICLE))
        if not sel.any():
            return []
        flat = np.nonzero(sel)[0]
        oid = ids[flat]
        rows, cols = flat // cam_w, flat % cam_w
        uniq, inv, counts = np.unique(oid, return_inverse=True, return_counts=True)
        n = len(uniq)
        x0 = np.full(n, 1e9); y0 = np.full(n, 1e9); x1 = np.zeros(n); y1 = np.zeros(n)
        np.minimum.at(x0, inv, cols); np.minimum.at(y0, inv, rows)
        np.maximum.at(x1, inv, cols); np.maximum.at(y1, inv, rows)
        out = []
        for i, k in enumerate(uniq):
            if counts[i] < 10:
                continue
            o = objs[k - 1]
            label = 'person' if o['kind'] == KIND_PERSON else 'car'
            conf = 0.80 + 0.18 * ((k * 7919) % 100) / 100.0
            out.append(dict(label=label, conf=round(conf, 2), box=(x0[i], y0[i], x1[i] + 1, y1[i] + 1),
                            source='SIM-GT'))
        return out

    def reset(self):
        self.drone.reset()

    def close(self):
        pass


def make_backend(kind='auto', host='127.0.0.1', vehicle='', camera='0', detect_meshes=None, use_infrared=False):
    if kind in ('auto', 'airsim'):
        try:
            b = AirSimBackend(host, vehicle, camera, detect_meshes, use_infrared)
            b.connect()
            return b
        except Exception as e:
            if kind == 'airsim':
                raise
            print(f'[bridge] AirSim not reachable at {host}:41451 ({type(e).__name__}: {e}).')
            print('[bridge] Falling back to the built-in MOCK simulator.')
    b = MockBackend()
    b.connect()
    return b
