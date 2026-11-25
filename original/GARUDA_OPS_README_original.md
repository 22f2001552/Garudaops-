# GARUDA OPS — Dataset-Driven Aerial Surveillance Dashboard

**CipherCop 2026 · Team Garuda Ops · Simulation-based prototype**

## Run on Windows

1. Install Python 3.10 or newer.
2. Extract this project ZIP.
3. Open a terminal in this folder.
4. Run `python server.py` (or `py server.py`).
5. Open **http://localhost:8000**.

**No pip install, npm install, or external Python dependencies are required.** The dashboard uses HTML/CSS/JavaScript and a Python standard-library HTTP server.

## Add your actual AirSim video

**Option A:** Click **Load AirSim video** in the dashboard and select your `.mp4` video. It stays in your browser; it is not uploaded to a server.

**Option B:** Rename/copy the video to `media/airsim.mp4` and restart the server. It loads automatically. The server supports HTTP byte-range video requests.

## How the dataset controls the dashboard

The sample dataset is `data/observations.json`. The dashboard synchronizes its metrics and map to the current video timestamp:

- `observations[]` holds sampled time points (`time_sec`), person/vehicle counts, crowd levels, and coordinates.
- `incidents[]` holds timestamped, geolocated observations for operator review.
- The nearest preceding observation determines the displayed counts at any timestamp.
- Incident cards and map markers appear when playback reaches each incident's timestamp.
- Crowd timeline is plotted directly from the dataset.
- Verify / Dismiss / Assign actions are **local demo state**; you can export an incident report with those statuses.
- You can replace the sample using **Import dataset** (JSON).

**IMPORTANT:** The bundled dataset is explicitly **synthetic and illustrative**. It has **not** been extracted from the user's AirSim video, and it is not YOLO inference output. Counts, alerts, and coordinates must not be represented as actual measurements from the video. There is no facial recognition or identity matching.

## Dataset schema

```json
{
  "schema_version": "1.0",
  "name": "My AirSim observations",
  "origin": "MANUALLY ANNOTATED FROM AIRSIM VIDEO",
  "video_filename": "airsim.mp4",
  "duration_sec": 120,
  "coordinate_note": "Coordinates from a documented simulation-to-map transform",
  "observations": [
    {"time_sec": 0, "person_count": 4, "vehicle_count": 1, "crowd_level": "LOW", "latitude": 17.4435, "longitude": 78.3772, "zone": "SECTOR A"},
    {"time_sec": 2, "person_count": 5, "vehicle_count": 1, "crowd_level": "LOW", "latitude": 17.4436, "longitude": 78.3773, "zone": "SECTOR A"}
  ],
  "incidents": [
    {"id": "OBS-001", "time_sec": 40, "type": "Crowd congestion", "severity": "HIGH", "latitude": 17.4436, "longitude": 78.3773, "zone": "SECTOR A", "description": "Operator review requested"}
  ]
}
```

## How to generate a *real* dataset from your AirSim video

1. Inspect the video and note its duration.
2. At regular intervals (e.g., every 2 seconds), count visible people/vehicles manually, or run a separate detector (YOLO/OpenCV) to produce timestamped counts. **No detector is included in this lightweight dashboard.**
3. Mark the timestamp of each important event. Avoid assuming congestion from counts alone without area calibration.
4. Add map coordinates only if simulation telemetry or a documented mapping exists. Otherwise use a schematic map and label positions illustrative.
5. Save the JSON in the schema above and import it, or replace `data/observations.json`.
6. Keep the video, JSON dataset, and this README together for the hackathon submission.

## Submission

- **Source code:** this entire project ZIP or the GitHub repository.
- **Dataset:** `data/observations.json` (labelled synthetic unless replaced with verified observations).
- **Presentation:** prepare a PPT/PDF with screenshots and explain video/dataset synchronization and limitations.

## GitHub

```bash
git init
git add .
git commit -m "Build dataset-driven Garuda Ops dashboard"
git branch -M main
git remote add origin https://github.com/vaishnavmore2004/cypercop.git
git push -u origin main
```

If `origin` already exists: `git remote set-url origin https://github.com/vaishnavmore2004/cypercop.git`.

## Security and limitations

This is a simulation research demo, not a deployed law-enforcement tool. The dashboard has no biometric recognition, real drone integration, live YOLO inference, actual dispatch integration, or verified GPS georeferencing. Operator actions are local to the current session. The map is schematic and does not depend on an internet tile server. Video is not included because no AirSim video was provided in this conversation.
