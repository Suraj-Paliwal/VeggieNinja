# okra_robot

Okra detection package for the robot: a YOLO11n segmentation model plus a sanity
filter that rejects things that can't be okra (rugs, legs, stools, poles).

## What's in here

```
okra_robot/
├── models/
│   ├── okra_seg.pt       PyTorch model (Kota0612/okra11n-seg-v5, base checkpoint)
│   └── okra_seg.onnx     same model for ONNX Runtime (no PyTorch needed)
├── okra_detector.py      OkraDetector class: model + filter, use this from robot code
├── okra_filter.py        the sanity checks and their thresholds
├── run_camera.py         live camera / video demo
├── export_model.py       build a TensorRT .engine on the robot (Jetson)
└── requirements.txt
```

## Setup on the robot

```bash
pip install -r requirements.txt
python run_camera.py --source 0            # prints okra centres for each frame
```

On an NVIDIA Jetson (Go2 EDU / G1 / H1 compute), install NVIDIA's JetPack PyTorch
build instead of the pip `torch`. Then build a TensorRT engine **on the Jetson
itself**. It is several times faster, and `OkraDetector` picks it up automatically:

```bash
python export_model.py engine              # creates models/okra_seg.engine
```

Model priority: `okra_seg.engine` → `okra_seg.onnx` → `okra_seg.pt`.

## Using it from your code

```python
from okra_detector import OkraDetector

det = OkraDetector()                       # stream=True: uses frame-to-frame stability
for frame in camera:                       # BGR uint8 images
    for okra in det.detect(frame):
        print(okra["cx"], okra["cy"], okra["conf"])
```

With a depth camera (RealSense etc.), pass a depth image in metres aligned to the
colour image, plus the colour camera intrinsics. The size check then uses real
length (5–25 cm) instead of a percentage of the frame, and you get a 3D point:

```python
okra = det.detect(frame, depth_m=depth, intrinsics=(fx, fy, ppx, ppy))
okra[0]["xyz_m"]      # (x, y, z) in metres, camera frame: x right, y down, z forward
okra[0]["length_m"]   # estimated pod length
```

Results are sorted best-first. `return_rejected=True` also returns what the filter
threw out, with a `reason`, which is useful when tuning.

## The sanity filter (okra_filter.py)

A detection counts as okra only if it passes all four:

| check     | rule                                                           | rejects            |
|-----------|----------------------------------------------------------------|--------------------|
| size      | 0.05–8 % of the frame, or 5–25 cm long when depth is given     | rug, big blobs     |
| shape     | long/short side ratio 2–7                                      | stools; poles      |
| colour    | ≥ 50 % of mask pixels okra-green (HSV hue 30–90)               | trousers, stools   |
| stability | seen near the same spot in 3 of the last 5 frames (streams)    | one-frame flickers |

All thresholds are constants at the top of `okra_filter.py`.

## Current status / limits (read before relying on it)

- **Tested on:** 4 still photos and one 3.5-minute indoor video (`session01_20260926_165754.mp4`)
  from the robot camera. On that video the filter cut detections from 3,831
  (in 2,898 frames) to 29 (in 27 frames). The video contains no real okra, so
  those 29 are false positives: a long green leaf, a patch of rug just under the size
  limit, and a sliver at the frame edge.
- **Not yet tested on robot footage that contains real okra.** Do that first to
  confirm real pods are kept.
- The filter can only remove detections. If the model misses a pod, the filter can't
  bring it back. Pods lying close together can be merged into one mask and rejected
  on shape.
- The model is the public base checkpoint. The earlier 13-image fine-tune performed
  worse and is not included. The real fix is fine-tuning on ~100–150 labeled
  frames from this camera, including "no okra" frames of the rug, legs and leaves.
- The pixel-based size limits assume a 640×480 camera about 0.3–1 m from the plant.
  Use depth on the robot so the size check is in real centimetres.
- Speed: ~8 FPS on a laptop CPU (i7-9750H) with the .pt model. Expect much faster
  with a TensorRT engine on a Jetson.
