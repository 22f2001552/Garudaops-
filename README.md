# GARUDA OPS × DRDO — Autonomous Obstacle Avoidance with Live Mission Control

This project integrates two earlier projects:

* **GARUDA OPS** (Vaishnav, CipherCop 2026): an aerial-surveillance dashboard.
* **Autonomous Obstacle Avoidance with Vision** (Manaswini, DYSL-AT / DRDO internship): depth-camera obstacle avoidance for a drone in Microsoft AirSim.

You run one Python server. It flies the drone in AirSim and serves the dashboard. **The dashboard controls what the drone does.** You can take off, land, trigger an E-stop, switch flight modes, fly it with sticks or the keyboard, click the map to send it somewhere, and tune the avoidance settings live. While it flies, the drone's camera is processed every frame and the results are streamed back to the dashboard.

```
 Browser (dashboard)  ── commands ──▶  server.py ── DroneController thread ──▶ AirSim (Unreal)
        ▲                                   │      depth → detect_obstacle_regions()      │
        └── telemetry + MJPEG sensor views ─┘      → rule-based avoidance → velocity cmd ◀┘
```

## What you see on the dashboard

| Panel | Contents |
|---|---|
| **Sensor feeds** | **Object detection overlay**: boxes around people and vehicles with their distance, boxes around depth obstacles, the L/C/R sectors, and an arrow showing the avoidance decision. **Depth heatmap**: near objects shown red, with a white contour at the threshold distance. **Crowd-density heatmap**: shows where people are concentrated. **Thermal visualization**: a heat image with °C readings and heat signatures marked. **Edges**: Canny edge detection, as described in report §3.4. **Quad view**: four of these views on one screen. |
| **Flight control** | Take off, land, and E-STOP. Modes: Manual, Autonomous, Patrol, Return home, and Hold. A stick pad plus keyboard control (W/A/S/D move, Q/E turn, R/F up/down, Space hover, T take off, L land, X E-stop). Obstacle assist and avoidance tuning sliders. |
| **Geospatial map** | The drone's live trail and camera view cone. A crowd heat layer and depth-mapped obstacles drawn on the ground. Incident pins. **Click the map to fly there; shift+click to build a patrol route.** |
| **Incident queue** | Avoidance manoeuvres, crowd congestion, low clearance, collisions and E-stops appear automatically. Each can be marked Verify, Dismiss or Assign, and you can export a report. |
| **Timeline / log** | Forward clearance and person count over the last 2 minutes, a table of detections, and the flight log. |
| **Replay** (`/replay`) | Vaishnav's original dashboard. Use **● Record flight** on the live page, then pick the recording here to replay its video and dataset, saved in his JSON format. |

## Run it (Windows)

1. `pip install -r requirements.txt`
2. Copy `airsim_config/settings.json` to `Documents\AirSim\settings.json`. This sets the camera "0" to 480×270 with a 90° field of view and enables Scene, Depth, and Infrared images.
3. Start your AirSim environment (Blocks, or any environment with people and cars) and press **Play**.
4. `python server.py`
5. Open **http://localhost:8000**, then click **TAKE OFF**, then **Autonomous** or **Patrol**, or click the map.

Without AirSim, `python server.py` falls back to a **built-in mock simulator**. The mock has buildings, a wall, trees, crowds of walking people and moving cars, so you can test the whole system on any PC. Use `--sim airsim` to require AirSim, or `--sim mock` to force the mock.

Useful options:

```
--detector yolo        # real YOLO detection on the AirSim camera (pip install ultralytics)
--detector airsim      # AirSim detection API (mesh-name filters, --detect-meshes "*Person*,*Car*")
--infrared             # use AirSim's Infrared image for the thermal view
--airsim-host IP       # AirSim on another machine
--camera 0 --vehicle Drone1
```

## How the DRDO code is used

* `original/obstacle_avoidance_original.py` is the script from report §3.8, unchanged.
* `drone/avoidance.py` contains `detect_obstacle_regions()` exactly as written in the report. It also has a non-blocking version of the report's rules:
  * centre clear → fly forward
  * centre blocked → shift to the clearer side, or rotate 30°/s if both sides are blocked
  * side blocked only → nudge ±0.5 m/s away from it
  * §5.2.6 forward-progress bias
  * §5.4 median filter, range cap, and low-pass smoothing
* What changed, and why:
  * The original loop blocked with `.join()` and `sleep()`, which would freeze the dashboard. The new version runs as a state machine at about 10 Hz.
  * Side-steps are now converted from the drone's own frame to the world frame. The original sent `vy` in the world frame, which is only correct while the drone faces north.
  * Optional ground-plane rejection stops the floor from being read as an obstacle at low altitude.
  * Cruise speed defaults to 2 m/s instead of 1 m/s. It can be changed with a slider.

## Honest limitations

* **Thermal:** in the mock, the thermal image comes from simulated temperatures. In AirSim without `--infrared`, it is a **pseudo-thermal** image built from brightness, depth and detections. Every frame is labelled with its source.
* **Detections:** in the mock, people and vehicle boxes come from the simulator's own ground truth (`SIM-GT`), not a trained model. Use `--detector yolo` for real model inference in AirSim. The Blocks environment has no people, so the crowd heatmap stays empty there. Use an environment that has people in it.
* **Coordinates:** detection positions on the map are estimated from the drone's pose, the pixel bearing and the depth, so they are approximate.
* **Purpose:** this is a simulation and research prototype. It has no facial recognition and no real drone or dispatch integration.

## Layout

```
server.py              HTTP server + API (/api/command, /api/telemetry, /stream/<view>, /api/sessions)
drone/avoidance.py     report algorithm (+ §5.4 preprocessing)
drone/controller.py    control loop, modes, incidents, map layers, recorder
drone/backends.py      AirSim backend + mock backend
drone/mock_world.py    built-in test world (ray-cast renderer)
drone/vision.py        detection overlay, depth/crowd/thermal/edge views
drone/detector.py      YOLO / AirSim detection API / sim ground truth
web/index.html, live.js, live.css    live mission control (new)
web/replay.html, replay.js, style.css  Vaishnav's dashboard (+ session picker)
original/              untouched originals for reference
```
