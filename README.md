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

## Requirements

* **Windows 10/11.** The prebuilt AirSim worlds are Windows builds.
* **Anaconda or Miniconda**, plus **Git**.
* **Python 3.9** in its own conda environment. The `airsim` package is unreliable on Python 3.11+.
* **About 2 GB free disk** for the AirSim Blocks world. A dedicated GPU helps, but Intel Iris Xe works with the settings below.

## Setup (tested step by step)

**1. Python environment**

```powershell
conda create -n garuda python=3.9 -y
conda activate garuda
git clone https://github.com/22f2001552/Garudaops-.git
cd Garudaops-
pip install numpy opencv-python
pip install tornado==4.5.3 msgpack-rpc-python
pip install airsim --no-build-isolation
```

`--no-build-isolation` is required. Without it, `pip install airsim` fails with `No module named 'numpy'`, because of a bug in the airsim package.

Check it worked: `python -c "import airsim, cv2, numpy; print('OK')"`

**2. AirSim simulator**

1. Download **Blocks.zip** (247 MB) from https://github.com/microsoft/AirSim/releases/tag/v1.8.1-windows and extract it.
2. Copy `airsim_config\settings.json` to `Documents\AirSim\settings.json`. **If your Documents folder is synced to OneDrive**, put it in `OneDrive\Documents\AirSim\` instead. AirSim prints the path it loaded on screen, so you can confirm.
3. Start it: `.\Blocks.exe -windowed -ResX=960 -ResY=540`, run from inside `Blocks\WindowsNoEditor`. Wait until the drone is visible.

**3. Run**

```powershell
python server.py --sim airsim
```

Open **http://localhost:8000**. The top bar should say **● AIRSIM** and show about 5–9 FPS. Click **TAKE OFF**, then **Autonomous** or **Patrol**, or click the map.

**No AirSim?** Run `python server.py --sim mock`. This uses the built-in test world, which only needs `numpy` and `opencv-python`.

**Two terminals help:** use one to start Blocks and one that stays in `Garudaops-` for the Python commands.

In the AirSim window, press **B** for a chase camera behind the drone, or **F1** for all view keys.

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

## Emergency response (crowd alert to police)

If the number of people near the drone (within `crowd_range`, 45 m) stays above the **crowd limit** for 3 seconds, the dashboard shows a popup: *"Crowd was too much in <area>… Send alert to the nearest police station? YES / NO"*. The popup includes a crowd-heatmap snapshot, the time to the millisecond, and the GPS position.

* The alert is **never sent automatically**. The operator decides, and the reaction time is logged.
* In the **Emergency Response** panel you can turn prompts on or off, set the crowd limit, and see every alert and its status.
* Snapshots and `alerts_log.json` are saved in `data/alerts/`.
* **Test alert** raises a clearly marked TEST alert. Use it in AirSim worlds that have no people.

## Tamper-evident recordings

Every view has an evidence strip under the picture showing the capture date and time to the millisecond, the recording clock (`REC 00:01:23.456`) and the frame number. These are burned into the saved video, so a cut shows up as a jump in the clock or frame number.

When you stop a recording, the server writes `data/<session>.manifest.json`. It contains the start and end times, every frame's timestamp, a SHA-256 hash chain over all frames, and the **SHA-256 fingerprints** of the video and dataset files. To check a recording, run:

```
python tools/verify_recording.py <session_name>     # or with no name, to check all recordings
```

It prints `VERIFIED` if the files are unchanged, or `TAMPERED` if even one byte differs. For evidence use, send the manifest (or its `video_sha256`) to the station at the end of the flight, so the fingerprint is kept somewhere the operator can't edit.

## Troubleshooting

| Problem | Fix |
|---|---|
| `pip install airsim` → `No module named 'numpy'` | `pip install airsim --no-build-isolation` |
| `TransportError: Retry connection over the limit` | AirSim isn't running or hasn't finished loading. Start Blocks, wait for the drone, then start the server. Check with `netstat -ano \| findstr 41451`. |
| Dashboard shows ~0.3 FPS | The depth image is too large. Use the repo's `settings.json`, which sets depth to 128×72, and restart Blocks. Run `python tools\airsim_benchmark.py` to time each call. |
| AirSim view shows a wall and no drone after take-off | Click the AirSim window and press **B** for the chase camera. |
| `ConnectionAbortedError` in the server terminal | Harmless: the browser cancelled a request. |
| `can't open file ...tools\airsim_benchmark.py` | You're in the Blocks folder. `cd` back into `Garudaops-`. |
| Dashboard says **MOCK SIM** but AirSim is open | Press **Play** in Unreal *before* starting `server.py`, and check that `pip install airsim msgpack-rpc-python` worked. Use `--sim airsim` to see the actual connection error. |
| `pip install airsim` fails | Install `msgpack-rpc-python` first, then `airsim`. Use Python 3.8–3.11 if a newer version fails. |
| Sensor feed stays on "WAITING FOR CAMERA STREAM" | Check the server console for errors. Make sure camera `"0"` exists in `settings.json`, or pass `--camera`. |
| Drone stops as if blocked by the ground | Keep **Ground rejection** ticked, or fly higher (above 4 m). |
| Drone goes back and forth in tight spaces | Lower the look-ahead threshold or the sidestep speed, or increase the avoid duration. |
| Recorded video won't play on the replay page | OpenCV could only write `mp4v`, which browsers don't play. Install an OpenCV build with H.264 support, or open the file in VLC. |
| Port 8000 already in use | `python server.py --port 8080` |

## API (for scripting or testing)

| Method | Path | Purpose |
|---|---|---|
| POST | `/api/command` | `{"action":"takeoff"}`, `land`, `estop`, `hold`, `{"action":"mode","mode":"PATROL"}`, `{"action":"goto","x":30,"y":10}`, `{"action":"set","params":{"forward_speed":3}}`, `{"action":"record","state":"start"}`, `stick`, `patrol_set`, `reset` |
| GET | `/api/telemetry?since=N` | Drone state, sectors, detections, map layers, events, log |
| GET | `/stream/<view>` | MJPEG stream: `detection`, `depth`, `crowd`, `thermal`, `edges`, `quad` |
| GET | `/snapshot/<view>.jpg` | A single frame |
| GET | `/api/sessions`, `/api/dataset?session=…` | Recorded flights in the GARUDA OPS dataset schema |

Example: `curl -X POST localhost:8000/api/command -d "{\"action\":\"takeoff\"}"`

## Credits

* **Obstacle avoidance:** Dharavath Manaswini Sanan, internship at DYSL-AT, DRDO Hyderabad, supervised by Shri Sumanth Pole, Scientist 'B'. Report: *Autonomous Obstacle Avoidance with Vision* (2025).
* **GARUDA OPS dashboard:** Vaishnav, Team Garuda Ops, CipherCop 2026.
* **Built on:** Microsoft AirSim, OpenCV, NumPy, and optionally Ultralytics YOLO.

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
