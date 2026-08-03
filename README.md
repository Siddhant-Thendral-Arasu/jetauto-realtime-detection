# JetAuto Perception — Real-Time Object Detection on a Jetson Robot

Real-time, GPU-accelerated object detection running on a Hiwonder JetAuto Pro (NVIDIA Jetson Orin Nano), built with ROS 2 Humble. A ROS 2 node subscribes to the robot's live camera feed and runs a TensorRT-optimized YOLO model on every frame, labeling objects in real time — fully on-device, no internet required at runtime.

> **Status:** working. Live detection runs on the robot's GPU. Open-vocabulary detection (YOLOE, text-promptable) is implemented and runs once its CLIP dependency is installed — see [Roadmap](#roadmap).

<!-- TODO: drop your demo clip/GIF here. Even a phone video of the screen works. -->
<!-- ![demo](media/demo.gif) -->

---

## What it does

- Subscribes to the robot's depth-camera RGB stream (`/depth_cam/rgb/image_raw`) over ROS 2.
- Runs a **TensorRT-compiled YOLO engine** on the Jetson GPU for each frame (real-time inference).
- Logs detected object classes live and writes annotated frames to disk (optional live display window).
- Runs **entirely offline** — the compiled engine ships with the model; no network needed to detect.

## Why it's built this way (design decisions)

- **TensorRT engine, not a raw `.pt` model.** On a Jetson, the win is edge-optimized inference. Loading a pre-compiled `.engine` through Ultralytics' `YOLO()` runs GPU-accelerated inference with minimal overhead — the same deployment pattern used in production robotics.
- **Two detection modes, chosen deliberately:**
  - **Fixed-vocabulary (this repo, default):** a TensorRT YOLO engine over the standard COCO classes. Fast, offline, reliable.
  - **Open-vocabulary (YOLOE):** text-promptable detection ("find the *mug*") without retraining — the more flexible, more current approach. Chosen over reflexively grabbing the newest YOLO release because *newest-that-works* beats *newest-that-exists* for a deployed system. Requires a one-time `clip` install (see Roadmap).
- **Own node, not the vendor demo.** The robot ships a detection example; this node was written from scratch (learning the engine-loading pattern from the vendor code, then implementing an independent node) so the pipeline is fully understood end to end.

## Architecture

```
[Depth camera] --/depth_cam/rgb/image_raw--> [detection node] --> annotated frames + class labels
                          (ROS 2 Image)            |
                                         YOLO TensorRT engine
                                          (Jetson GPU inference)
```

## Hardware / software

| | |
|---|---|
| Robot | Hiwonder JetAuto Pro |
| Compute | NVIDIA Jetson Orin Nano |
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| Inference | PyTorch 2.7 (CUDA), Ultralytics, TensorRT |
| Camera | Orbbec Astra Pro Plus (depth cam, RGB stream used) |

## Repository layout

```
nodes/
  my_detect.py        # TensorRT YOLO detection node (offline, fixed-vocab) — the working node
  live_detect.py      # open-vocab YOLOE detection node (needs `clip`)
docs/
  DEBUGGING.md         # engineering log: the hard problems and how they were solved
README.md
.gitignore
```

## Running it

On the robot (ROS 2 Humble sourced, camera driver up):

```bash
python3 nodes/my_detect.py
```

Then place objects in front of the camera — detected classes stream to the console and annotated frames are written to `~/my_detected.jpg`. For a live window, uncomment the `cv2.imshow` lines in the callback.

<!-- Note: paths to the .engine file are set at the top of my_detect.py; adjust to your model location. -->

## Roadmap

- [x] Live TensorRT YOLO detection on the robot's camera (offline)
- [ ] Open-vocabulary detection (YOLOE + CLIP) — implemented in `live_detect.py`, pending `clip` install
- [ ] Detection-driven behavior (center on / approach a named object)
- [ ] Language-conditioned control: natural-language instruction → VLM planner → navigation/manipulation (the Nav2 stack and a vendor VLM+camera example are already on the platform)

## Engineering log

The interesting part of embedded robotics is everything that goes wrong. See **[docs/DEBUGGING.md](docs/DEBUGGING.md)** for the real problems solved along the way — a corrupted ROS 2 environment, an IPv6-only DHCP failure, and pivoting to an offline TensorRT engine when a dependency couldn't be downloaded.

## License

MIT — see [LICENSE](LICENSE).
