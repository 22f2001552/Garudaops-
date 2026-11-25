"""Object detection sources for the overlay / crowd / thermal views.

  sim     – mock simulator ground truth (id map)          [mock only]
  airsim  – AirSim detection API (mesh-name filters)       [AirSim >= 1.5]
  yolo    – Ultralytics YOLO on the RGB frame (optional: pip install ultralytics)
  none    – depth-based obstacle boxes only

'auto' picks: mock -> sim; AirSim -> yolo if ultralytics is installed, else the
AirSim detection API.  Every source returns the same dict format:
  {label, conf, box:(x0,y0,x1,y1) in pixels, source}
"""
COCO_KEEP = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}
VEHICLES = {'car', 'bus', 'truck', 'motorcycle', 'bicycle'}


class Detector:
    def __init__(self, mode, backend, yolo_weights='yolov8n.pt', every=2, conf=0.35):
        self.backend = backend
        self.every = max(1, int(every))
        self.conf = conf
        self.model = None
        self._n = 0
        self._cache = []
        if mode == 'auto':
            mode = 'sim' if backend.name.startswith('MOCK') else 'yolo'
        if mode == 'sim' and not backend.name.startswith('MOCK'):
            mode = 'airsim'
        if mode == 'yolo':
            try:
                from ultralytics import YOLO
                self.model = YOLO(yolo_weights)
                print(f'[detector] YOLO loaded ({yolo_weights})')
            except Exception as e:
                print(f'[detector] YOLO unavailable ({type(e).__name__}: {e}); '
                      'using simulator detections instead.')
                mode = 'sim' if backend.name.startswith('MOCK') else 'airsim'
        self.mode = mode

    @property
    def label(self):
        return {'sim': 'SIM GROUND-TRUTH', 'airsim': 'AIRSIM DETECTION API',
                'yolo': 'YOLO (ULTRALYTICS)', 'none': 'DEPTH OBSTACLES ONLY'}.get(self.mode, self.mode.upper())

    def detect(self, scene):
        h, w = scene.shape[:2]
        if self.mode == 'none':
            return []
        if self.mode in ('sim', 'airsim'):
            return self.backend.detections(w, h)
        # yolo — run every N frames, reuse in between
        self._n += 1
        if self._n % self.every and self._cache is not None:
            return self._cache
        res = self.model.predict(scene, imgsz=max(w, h), conf=self.conf, verbose=False,
                                 classes=list(COCO_KEEP))[0]
        out = []
        for b in res.boxes:
            c = int(b.cls[0])
            lab = COCO_KEEP.get(c, 'object')
            x0, y0, x1, y1 = (float(v) for v in b.xyxy[0])
            out.append(dict(label='car' if lab in VEHICLES and lab != 'bicycle' else lab,
                            conf=round(float(b.conf[0]), 2), box=(x0, y0, x1, y1), source='YOLO'))
        self._cache = out
        return out
