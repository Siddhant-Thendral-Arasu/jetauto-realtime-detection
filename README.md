# JetAuto Perception, Autonomy & Manipulation

A Hiwonder JetAuto Pro (NVIDIA Jetson Orin Nano) that sees, tracks, drives toward objects, and moves its arm to points in 3D space, all on-device. It runs ROS 2 Humble, does its inference with a TensorRT-compiled YOLO model, and needs no internet at runtime.

I started with plain object detection and kept going: detect → turn to face a target → drive up to it using the depth camera → bring up the arm → locate objects in 3D → solve the arm's kinematics myself. Each step builds on the last.

> **Status:** detection, tracking, depth-based approach, arm control, and inverse kinematics all run on the real robot. Next up: connecting the camera's 3D estimate to the arm so it reaches for what it sees.

<!-- DEMO CLIP: drag a 20–30s video or GIF into this spot on GitHub's web editor -->

## What it does

**Perception and driving**
- **`my_detect.py`**: the detector. TensorRT engine, COCO classes, runs on the Jetson's GPU in real time.
- **`track_node.py`**: turns the robot to keep a named object centered in frame (proportional control on `/cmd_vel`).
- **`approach_node.py`**: drives toward the target and stops at a set distance, reading real meters off the depth camera. Lose the target and it sweeps to reacquire, giving up after a full rotation.

**The arm**
- **`arm_control.py` / `arm_home.py`**: per-joint control of the 6-servo arm (5 joints + gripper) and named poses.
- **`grasp_sequence.py`**: a scripted ready → reach → close → lift grab.
- **`fetch_node.py`**: an end-to-end see → drive → grab prototype. It failed, and the reason is the most useful thing in this repo (below).

**3D and kinematics**
- **`locate_3d.py`**: turns a detection plus its depth reading into an (X, Y, Z) point in meters, using the camera's intrinsics.
- **`forward_kinematics.py`**: given joint angles, where the gripper tip ends up. Built from the robot's own URDF.
- **`inverse_kinematics.py`**: given a target point, the joint angles that reach it. Solved analytically, no MoveIt.

**Measurement**
- **`bench.py`**: benchmarks the detector on the robot (TensorRT vs. PyTorch, plus camera-to-detection latency).

## Performance

Measured on the Jetson Orin Nano with `bench.py` (yolo26n, same 100 camera frames through both models):

| | Per frame | Throughput |
|---|---|---|
| **TensorRT engine** | **18.1 ms** | **55 FPS** |
| PyTorch (.pt, GPU) | 35.7 ms | 28 FPS |

TensorRT is **2.0× faster**. On the live camera stream, a frame's detections are ready a median **45 ms** after capture. The live loop runs at the camera's ~15 FPS, so the camera, not the model, is the bottleneck.

## Decisions taken

**TensorRT engine instead of a raw `.pt` file.** On a Jetson the whole point is edge-optimized inference, and the benchmark above shows what it buys: half the latency of plain PyTorch on the same GPU.

**Depth camera for distance, not a bounding-box trick.** My first approach node guessed distance from box height. It worked until I stood close enough to fill the frame, maxing out the box height so the robot thought I was far and never stopped. Sampling the actual depth stream fixed it, for any object, however it's framed.

**I don't stop the instant the target disappears.** Detection flickers. A few missed frames are tolerated, and when the target really is lost the robot keeps turning toward where it last saw it before searching.

**Kinematics by hand instead of MoveIt.** The arm is a base yaw plus three pitch joints in one vertical plane, with a wrist roll that spins the gripper without moving its tip. That structure has a clean analytic solution: the yaw aims the plane at the target, then the reach and height reduce to a two-link triangle solved with the law of cosines once you fix the gripper's approach angle. I pulled every link length from the URDF, wrote forward kinematics first, and built inverse kinematics by inverting it. FK doubles as the answer key: every IK solution is fed back through FK and rejected if the tip misses. Round-trip error is 0.0 mm, and the physical arm lands where the model says.

**My own nodes, not the vendor demo.** I read the vendor's examples to learn the patterns, then wrote these from scratch so I understood every piece, which mattered a lot when things broke.

## The failed prototype (and why it matters)

`fetch_node.py` tried to do it all in one pass: detect a bottle, drive to it, and trigger the grab. It never grabbed reliably. The cause: the camera has a **~0.5 m blind spot** in front of the robot. Once the bottle got close enough to grab, it dropped out of view, so the final approach was driving blind.

The fix is what the 3D and kinematics work above is for: see the object **once** from a distance, convert it to a fixed point in space with `locate_3d`, then let IK drive the arm to that point. The blind spot stops mattering once you're reaching for a stored coordinate instead of chasing live pixels.

## How the pieces fit together

```
my_detect.py         perception    load the engine, run inference on a frame
   ├── track_node.py     behavior    turn to center the target
   │      └── approach_node.py      + depth, drive toward it, search if lost
   └── locate_3d.py      3D          detection + depth -> (X, Y, Z) in meters
                                          │
forward_kinematics.py  ◄──checks──  inverse_kinematics.py  -> servo targets -> arm
```

## Hardware / software

| | |
|---|---|
| Robot | Hiwonder JetAuto Pro (mecanum base + 5-DOF arm + gripper) |
| Compute | NVIDIA Jetson Orin Nano |
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| Inference | TensorRT, Ultralytics YOLO, PyTorch (CUDA) |
| Camera | Orbbec Astra Pro Plus (RGB + depth) |

## Running it

Source ROS 2 and bring the camera up first. For anything that moves the arm, also run the servo controller (`ros2 run ros_robot_controller ros_robot_controller`). Then:

```bash
python3 nodes/my_detect.py         # detection
python3 nodes/approach_node.py     # detect + turn + drive toward it
python3 nodes/locate_3d.py         # print the target's 3D position
python3 nodes/inverse_kinematics.py 0.30 0 0.44 72   # servo values for a target (m) + approach angle
python3 nodes/bench.py             # TensorRT vs. PyTorch benchmark
```

Set the target class and the engine path at the top of each file. Keep all the nodes in the same directory; they import each other.

## What's next

- [x] Real-time TensorRT detection on the robot (offline)
- [x] Turn to center a named object
- [x] Depth-based approach with search-and-reacquire
- [x] Arm bring-up and scripted grasp
- [x] Detection + depth → 3D position
- [x] Forward and inverse kinematics, verified on the arm
- [ ] Camera → arm transform, so the arm reaches for what the camera sees
- [ ] Full pick-up: see it, drive close, reach, grab
- [ ] Open-vocab detection (YOLOE + CLIP)

## The messy part

Most of the real work wasn't the ML: a corrupted ROS 2 environment, a DHCP failure that only handed out IPv6, and "the robot won't move" traced through the whole ROS graph to a motor controller crashing on a loose power cable. See **[docs/DEBUGGING.md](docs/DEBUGGING.md)**.

The arm added one more: commands that logged as sent but never moved it. The node was publishing before the servo controller had connected, so the messages went nowhere. The fix was waiting for a subscriber before publishing.

## License

MIT, see [LICENSE](LICENSE).
