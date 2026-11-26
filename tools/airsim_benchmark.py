"""Times each AirSim call the GARUDA server makes, to find what is slow.

Run with AirSim (Blocks) open and the drone visible, and with server.py STOPPED:
    python tools/airsim_benchmark.py
"""
import statistics
import time

import airsim

c = airsim.MultirotorClient()
c.confirmConnection()
A = airsim.ImageType


def bench(name, fn, n=5):
    times = []
    for _ in range(n):
        t = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t) * 1000)
    print(f'{name:<38} {statistics.median(times):8.0f} ms  (min {min(times):.0f}, max {max(times):.0f})')


print('\nTiming AirSim calls (median of 5)...\n')
bench('ping', c.ping)
bench('getMultirotorState', c.getMultirotorState)
bench('simGetCollisionInfo', c.simGetCollisionInfo)
bench('scene image only', lambda: c.simGetImages([airsim.ImageRequest('0', A.Scene, False, False)]))
bench('depth image only', lambda: c.simGetImages([airsim.ImageRequest('0', A.DepthPerspective, True, False)]))
bench('scene + depth (what server uses)', lambda: c.simGetImages([
    airsim.ImageRequest('0', A.Scene, False, False),
    airsim.ImageRequest('0', A.DepthPerspective, True, False)]))
try:
    c.simSetDetectionFilterRadius('0', A.Scene, 120 * 100)
    c.simAddDetectionFilterMeshName('0', A.Scene, '*Person*')
    bench('simGetDetections', lambda: c.simGetDetections('0', A.Scene))
except Exception as e:
    print('simGetDetections                       unavailable:', e)

r = c.simGetImages([airsim.ImageRequest('0', A.Scene, False, False)])[0]
print(f'\nCamera image size: {r.width} x {r.height}  (expected 480 x 270 from settings.json)')
print('Done. Send these numbers back.')
