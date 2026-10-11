# nodes/

Every script that runs on the robot. They import each other, so keep them all in this one folder.

## Before running anything

In a clean shell (see [DEBUGGING.md](../docs/DEBUGGING.md) for why it has to be clean), source ROS 2 and start what the script needs:

```bash
bash --norc --noprofile
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch peripherals depth_camera.launch.py          # camera: needed by anything that sees
ros2 run ros_robot_controller ros_robot_controller      # servo/motor board: needed by anything that moves
```

## Perception

| File | What it does | Needs |
|---|---|---|
| `my_detect.py` | The base detector: loads the TensorRT YOLO engine, runs it on each camera frame, logs what it sees. Also exports `load_detector()` and `detect()` for the other nodes. | Camera |
| `live_detect.py` | Open-vocabulary detection with YOLOE: finds objects by text prompt instead of a fixed class list. Needs the `clip` library installed. | Camera |
| `yolo_test.py` | Runs YOLOE once on a saved image. A quick check that the model loads and detects. | Nothing |
| `locate_3d.py` | Turns a detection into a 3D point in meters: samples the depth image inside the box (nearest surface, so a thin bottle beats the wall behind it) and back-projects with the camera's intrinsics. | Camera |
| `bench.py` | Benchmarks the detector: TensorRT vs. PyTorch on the same frames, plus live camera-to-detection latency. | Camera |

## Driving

| File | What it does | Needs |
|---|---|---|
| `track_node.py` | Turns the base to keep a named object centered in frame (proportional control on `/cmd_vel`). | Camera, controller |
| `approach_node.py` | Turns toward the target and drives up to it, stopping at a set distance read from the depth camera. If the target is lost, it sweeps to reacquire and gives up after a full rotation. | Camera, controller |

## Arm

| File | What it does | Needs |
|---|---|---|
| `arm_control.py` | Command-line arm control: named poses, gripper open/close, single-joint jogs. | Controller |
| `arm_home.py` | Sends the arm to its straight-up home pose. | Controller |
| `grasp_sequence.py` | A scripted grab: ready → reach → close → lift. | Controller |
| `fetch_node.py` | **Failed prototype:** detect a bottle, drive to it, then run `grasp_sequence.py`. The camera can't see the last ~0.5 m in front of the robot, so the final approach was blind. Kept for the record; see the main README. | Camera, controller |

## Kinematics

| File | What it does | Needs |
|---|---|---|
| `forward_kinematics.py` | Servo positions → where the gripper tip ends up (x, y, z). Link lengths and axes come from the robot's URDF. | Nothing |
| `inverse_kinematics.py` | Target point (x, y, z) + approach angle → the servo positions that reach it. Solved analytically, then checked by running the answer back through forward kinematics. | Nothing |

## Quick reference

```bash
python3 my_detect.py                          # detection
python3 approach_node.py                      # detect + turn + drive toward it
python3 locate_3d.py                          # print the target's 3D position

python3 arm_control.py home                   # named pose (home / ready)
python3 arm_control.py open                   # gripper open / close
python3 arm_control.py jog 2 350              # move one servo (id, position 0–1000)
python3 grasp_sequence.py                     # scripted grab

python3 forward_kinematics.py 500 350 500 500 500   # servos 1–5 -> tip position
python3 inverse_kinematics.py 0.30 0 0.44 72        # target (m) + approach angle -> servos

python3 bench.py                              # TensorRT vs. PyTorch benchmark
```

## Arm reference

| Servo ID | Joint | Notes |
|---|---|---|
| 1 | Base rotation | |
| 2 | Shoulder | |
| 3 | Elbow | |
| 4 | Wrist tilt | |
| 5 | Wrist rotate | Spins the gripper; doesn't move the tip |
| 10 | Gripper | 200 = open, 800 = closed |

Servo range is 0–1000, with **500 = straight up** for joints 1–5 (about 0.24° per unit). Below 500 leans joints 2–4 forward and down.

## How they depend on each other

```
my_detect ─┬─ track_node ─┬─ approach_node
           │              └─ fetch_node ── runs grasp_sequence
           ├─ locate_3d
           └─ bench
forward_kinematics ── inverse_kinematics
```

Set the target class and the model/engine path at the top of each file before running.
