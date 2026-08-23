# Engineering log

Embedded robotics is mostly debugging. These are the non-trivial problems I hit building this, and how I actually found and fixed them.

## 1. A corrupted ROS 2 environment that broke every source

**Symptom.** Every new terminal, `source /opt/ros/humble/setup.bash` threw:

```
setup.bash:.:11: no such file or directory: /home/ubuntu/setup.sh
```

The path was resolving against my current directory instead of the install tree, so nothing sourced right and I couldn't launch a single ROS node normally.

**Dead ends.** I assumed a corrupted workspace first and rebuilt it (`rm -rf build install log && colcon build`). No change. Then I figured the base ROS install's setup files were missing, so I copied replacements in and ran `apt install --reinstall ros-humble-ros-base`. Still broke.

**The test that cracked it.** I sourced inside a shell that skips all startup scripts:

```bash
bash --norc --noprofile -c 'source /opt/ros/humble/setup.bash && echo CLEAN_OK'
```

It printed `CLEAN_OK`. That was the whole answer: the ROS setup file was fine. The bug lived in a shell startup script that runs in normal terminals. It was mis-sourcing ROS and polluting the environment so every later source resolved paths wrong.

**Fix.** Run ROS commands in a clean-but-full shell (`bash --norc --noprofile`, then source with absolute paths). That dodges the broken startup script while keeping the full environment vendor launch files need. The permanent fix is correcting that startup script's source line to use absolute paths.

**Takeaway.** When a reinstall doesn't fix a "missing file" error, the file usually isn't the problem, rather, the thing invoking it is. A no-startup shell isolated that in one command, after two rebuilds got me nowhere.

## 2. WiFi "connected" but no internet, an IPv6-only DHCP lease

**Symptom.** The WiFi panel said Connected. `ping github.com` failed with Destination Host Unreachable, routing through a dead interface.

**Diagnosis.** `ip -brief address show wlan0` showed only IPv6 addresses on the interface, no IPv4 at all. `ip route` showed the default route pinned to a dead USB-tether bridge (`l4tbr0`, linkdown). So anything IPv4, like GitHub, had no valid route out. The router had handed out an IPv6 lease and no IPv4.

**Fix.** Bounce the connection to force a fresh DHCP lease that includes IPv4:

```bash
sudo nmcli connection down <SSID>
sudo nmcli connection up <SSID>
ip -brief address show wlan0   # confirm a 192.168.x.x shows up
```

**Takeaway.** "Connected" is not "has a working route." Reading the interface addresses and the routing table directly turned a vague "no internet" into a specific, fixable cause.

## 3. Couldn't download a dependency, so I went offline

**Situation.** The open-vocab detector (YOLOE) needs the `clip` library at runtime, installed via pip. The robot's network kept failing to hand out an IP, so I couldn't install it. Blocked on the last step.

**The pivot.** Instead of fighting the network, I checked what was already on the machine and found pre-compiled TensorRT YOLO engines (`yolo26n.engine`, `yolov11n.engine`) shipped with the platform. TensorRT engines run fully offline and don't touch CLIP. I read the vendor's detection node to learn the engine-loading pattern, then wrote my own node around it, pointed at the live camera. Real-time GPU detection, working, zero internet.

Two small bugs on the way:

- `self.model.names[...]` fails on a TensorRT engine model, class names are exposed differently. The fix was reading them off the results object: `results[0].names[...]`.
- `self.get_logger.info(...)` — `get_logger` is a method. Needs `self.get_logger().info(...)`.

**Takeaway.** When an external dependency blocks you, inventory what's already on the machine. The offline TensorRT path turned out to be more deployment-relevant than the original plan, not a downgrade.

## 4. "The robot won't move" — a full ROS-graph chase that ended at a loose cable

This is the one I'd tell in an interview. It took a while and the cause was almost stupidly physical, but the process is the point.

**Symptom.** After a reboot, my nodes ran fine, detection logging, depth reading real meters, no errors, but the wheels did nothing. Publishing to `/cmd_vel` by hand did nothing either.

**Working down the stack.** I didn't want to guess, so I checked one layer at a time.

- Was anything even listening? `ros2 topic info /cmd_vel` showed one subscriber: `odom_publisher`. That node *reads* velocity commands to estimate position. It doesn't drive motors. So my commands were landing, but the thing that turns wheels wasn't in the graph at all.
- `ros2 node list` confirmed it: `ros_robot_controller`, the actual motor driver, wasn't running.
- So why not? I checked the serial devices, `/dev/ttyACM*` and `/dev/ttyUSB*` were present, so the board was visible. The controller package was installed. Both the software and the hardware looked fine.
- I read the boot log for the startup service: `journalctl -u start_app_node.service -b`. The controller launched, tried its first serial write to the motor board, and died:

```
serial.serialutil.SerialException: write failed: [Errno 5] Input/output error
process has died [pid ..., exit code 1]
```

**The actual cause.** `Errno 5` on a serial *write* means the port opened but the hardware link failed mid-write. Not software. That pointed at the physical connection, and sure enough the base's power cable had worked loose. A loose power connector browns out the controller board under load, which is exactly why the failure was intermittent and why it also explained a separate "works, then gets jittery, then dies" problem I'd been chasing. One loose cable was behind all of it.

**Fix.** Seat the cable, restart the service, confirm `ros_robot_controller` stays alive and now subscribes to `/cmd_vel`. Wheels turned.

**Takeaway.** Intermittent + physical-smelling almost always means power or a connection, not code. Walking the ROS graph top-down (who's subscribed → what's running → why did it die → what does the log say) turned "it won't move" into a specific one-line log entry, and the log entry named the real problem. I'd been about to rewrite node code that was never broken.

## Notes on process

Most of these took longer than they "should" have. That's normal for embedded work on a vendor-customized image where the docs don't match the shipped system. The pattern that kept working: stop guessing, isolate one variable at a time, and read the system's actual state, `ip`, a no-startup shell, `journalctl`, `ros2 topic info`, instead of assuming where the problem is. Every one of these bugs I initially blamed on the wrong layer. Checking state instead of guessing is what corrected that, every time.
