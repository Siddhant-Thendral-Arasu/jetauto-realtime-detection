# Engineering log

Embedded robotics is mostly debugging. These are the non-trivial problems I hit building this and how I diagnosed and solved them. I'm including this because *how* someone debugs matters more than a clean final result — the messy middle is where the real work is.

---

## 1. A corrupted ROS 2 environment that broke every `source`

**Symptom:** Every new terminal, `source /opt/ros/humble/setup.bash` threw:
```
setup.bash:.:11: no such file or directory: /home/ubuntu/setup.sh
```
The path resolved against my current directory instead of the install tree, so nothing sourced correctly — I couldn't launch any ROS node normally.

**Dead ends:** I first assumed a corrupted workspace and rebuilt it (`rm -rf build install log && colcon build`) — no change. I then assumed the base ROS install's setup files were missing and copied replacements from the workspace, and even ran `apt install --reinstall ros-humble-ros-base`. Still broke.

**The decisive test:** I ran the source inside a shell that skips all startup scripts:
```bash
bash --norc --noprofile -c 'source /opt/ros/humble/setup.bash && echo CLEAN_OK'
```
It printed `CLEAN_OK`. That proved the ROS setup file itself was fine, and the bug lived in a **shell startup script** that runs in normal terminals — it mis-sourced ROS and polluted the environment so later sources resolved paths incorrectly.

**Fix / workaround:** run ROS commands in a clean-but-full shell (`bash --norc --noprofile`, then source with absolute paths). This dodges the broken startup script while keeping the full environment that vendor launch files need. The permanent fix is correcting that startup script's source line to use absolute paths.

**Takeaway:** when a reinstall doesn't fix a "missing file" error, the file probably isn't the problem — the *environment invoking* it is. Isolating variables (a no-startup shell) beat guessing.

---

## 2. WiFi "connected" but no internet — an IPv6-only DHCP lease

**Symptom:** The WiFi panel showed *Connected*, but `ping github.com` failed with `Destination Host Unreachable`, routing through a dead interface.

**Diagnosis:** `ip -brief address show wlan0` showed the interface had **only IPv6 addresses, no IPv4**. `ip route` showed the default route pinned to a dead USB-tether bridge (`l4tbr0`, `linkdown`). So IPv4 traffic (like GitHub) had no valid route — the router had handed out an IPv6 lease but no IPv4.

**Fix:** bounce the connection to force a fresh DHCP lease that includes IPv4:
```bash
sudo nmcli connection down <SSID>
sudo nmcli connection up <SSID>
ip -brief address show wlan0   # confirm a 192.168.x.x appears
```

**Takeaway:** "Connected" is not "has a working route." Reading the interface addresses and the routing table directly (`ip address`, `ip route`) turned a vague "no internet" into a specific, fixable cause.

---

## 3. Couldn't download a dependency — so I went offline

**Situation:** The open-vocabulary detector (YOLOE) needs the `clip` library at runtime, installed via pip. The robot's network kept failing to hand out an IP, so I couldn't install it — blocked on the last step.

**The pivot:** Instead of fighting the network, I checked what was already on the machine and found pre-compiled **TensorRT YOLO engines** (`yolo26n.engine`, `yolov11n.engine`) shipped with the platform. TensorRT engines run fully offline and don't need CLIP (no text prompts). I read the vendor's detection node to learn the engine-loading pattern, then **wrote my own node** around it — pointed at the live camera, my own structure. Result: real-time GPU detection, working, with zero internet.

**Two small bugs fixed along the way:**
- `self.model.names[...]` fails on a TensorRT engine model (class names are exposed differently) — the fix was reading names off the results object: `results[0].names[...]`.
- `self.get_logger.info(...)` → `get_logger` is a method; it needs `self.get_logger().info(...)`.

**Takeaway:** when blocked by an external dependency, inventory what you already have. The offline TensorRT path turned out to be a *more* deployment-relevant result than the original plan, not a downgrade.

---

## Notes on process

Most of these took longer than they "should" have — that's normal for embedded work on a vendor-customized image where the docs don't always match the shipped system. The pattern that consistently worked: stop guessing, isolate one variable at a time, and read the system's actual state (`ip`, a no-startup shell, `ls` on the real paths) instead of assuming.
