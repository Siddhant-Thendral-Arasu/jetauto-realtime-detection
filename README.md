# JetAuto Perception & Autonomy

A Hiwonder JetAuto Pro (NVIDIA Jetson Orin Nano) that sees, tracks, and drives toward objects on its own. It runs ROS 2 Humble, does all of its inference on-device with a TensorRT-compiled YOLO model, and needs no internet at runtime.

I started with plain object detection and kept going: detect → turn to face a target → drive up to it using the depth camera. Each node builds on the last.

> **Status:** working on real hardware. Detection, visual tracking, and depth-based approach all run on the robot. Open-vocabulary detection (YOLOE) is written and runs once its CLIP dependency is installed — more on that below.

## What it does

The robot subscribes to its own camera over ROS 2 and runs YOLO on every frame, on the Jetson's GPU, in real time. That's the base. On top of it:

- **`my_detect.py`** — the detector. TensorRT engine, COCO classes, writes annotated frames and logs what it sees. Offline.
- **`track_node.py`** — turns the robot to keep a named object centered in frame. Proportional control on `/cmd_vel`.
- **`approach_node.py`** — drives toward the target and stops at a set distance, reading real meters off the depth camera. Includes a search behavior: lose the target and it sweeps to reacquire, giving up after a full rotation.

Everything past detection is a control loop: camera in, velocity command out, repeat.

## Decisions Taken

**TensorRT engine instead of a raw `.pt` file.** On a Jetson the whole point is edge-optimized inference. Loading a pre-compiled `.engine` through Ultralytics runs GPU-accelerated with almost no overhead. It's how you'd actually deploy this, not just how you'd prototype it.

**Depth camera for distance, not a bounding-box trick.** My first approach node guessed distance from how tall the person's box was. It worked until I found out standing close enough that you fill the frame and maxes out the box height, so the robot thinks you're far and never stops. The fix was to sample the actual depth stream at the target's center and work in meters. No saturation, and it works for any object regardless of how it's framed.

**I don't stop the instant the target disappears.** Detection flickers. Drop into "lost" mode on a single missed frame and the robot fidgets constantly. So a few missed frames are tolerated before it reacts, and when it does lose the target it keeps turning toward where it last saw it before searching. Small thing, big difference in how it feels.

**My own nodes, not the vendor demo.** The robot ships with a detection example. I read it to learn the engine-loading pattern, then wrote these from scratch so I understood every piece, which mattered a lot when things broke.

## How the nodes fit together

```
my_detect.py      perception    load the engine, run inference on a frame
      |
track_node.py     behavior      find the target, turn to center it
      |
approach_node.py  behavior      + read depth, drive toward it, search if lost
```

`track_node` imports `my_detect`; `approach_node` imports both. Keep all three in the same directory.

```
[depth camera] --/depth_cam/rgb/image_raw--> [node] --/cmd_vel--> [motors]
                                               |
                                        YOLO TensorRT engine (Jetson GPU)
```

## Hardware / software

| | |
|---|---|
| Robot | Hiwonder JetAuto Pro |
| Compute | NVIDIA Jetson Orin Nano |
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| Inference | PyTorch (CUDA), Ultralytics, TensorRT |
| Camera | Orbbec Astra Pro Plus (RGB + depth) |

## Layout

```
nodes/
  my_detect.py       detector (offline, fixed-vocab) — the base node
  track_node.py      turn-to-center behavior
  approach_node.py   depth-based approach + search
  live_detect.py     open-vocab YOLOE detector (needs clip)
docs/
  DEBUGGING.md       the log of everything that went wrong and how I fixed it
README.md
```

## Running it

Source ROS 2 and bring the camera up first, then:

```bash
python3 nodes/my_detect.py       # just detection
python3 nodes/track_node.py      # detect + turn to face a target
python3 nodes/approach_node.py   # detect + turn + drive toward it
```

Set the target class and the engine path at the top of each file. For approach, run it on the floor with room to move and a hand near Ctrl-C the first time.

## What's next

- [x] Real-time TensorRT detection on the robot (offline)
- [x] Turn to center a named object
- [x] Depth-based approach with a search-and-reacquire behavior
- [ ] Open-vocab detection (YOLOE + CLIP) — written, waiting on a `clip` install
- [ ] Grab it: 3D position from depth → inverse kinematics → the arm picks it up
- [ ] Natural-language control: say what you want, the robot plans and does it

## The messy part

Most of the real work here wasn't the ML, but a corrupted ROS 2 environment, a DHCP failure that only handed out IPv6, and chasing "the robot won't move" through the entire ROS graph only to find the motor controller was crashing on a loose power cable. That's all written up in **[docs/DEBUGGING.md](docs/DEBUGGING.md)**.

## License

MIT — see [LICENSE](LICENSE).
