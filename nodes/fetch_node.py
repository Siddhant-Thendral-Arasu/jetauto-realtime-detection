import os
import time
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from my_detect import load_detector, detect
from track_node import turn, KP, MAX_TURN

TARGET = 'bottle'
DETECT_CONF = 0.25
ALIGN_TOL = 120
FWD_SPEED = 0.1
TRIGGER_Y_FRAC = 0.72
TRIGGER_DIST = 0.70
CENTER_TOL = 110
LOST_GRAB_DIST = 0.90
LOST_CENTER_TOL = 160
CREEP_SPEED = 0.07
GRAB_OFFSET = 0.20
DEFAULT_REMAIN = 0.30
PATCH = 5
SEARCH_START = 0.35
SEARCH_ACCEL = 0.30
SEARCH_MAX = 0.60
SEARCH_LIMIT = 6.28

GRASP_SCRIPT = os.path.expanduser('~/dev_ws/grasp_sequence.py')

def find_target_box(results, target):
	if not results:
		return None
	r = results[0];
	best, best_area = None, 0.0
	for b in r.boxes:
		if r.names[int(b.cls)] != target:
			continue
		x1, y1, x2, y2 = b.xyxy[0].tolist()
		area = (x2 - x1) * (y2 - y1)
		if area > best_area:
			best_area, best = area, (x1, y1, x2, y2)
	return best
	
class FetchNode(Node):
	def __init__(self):
		super().__init__('fetch_node')
		self.bridge = CvBridge()
		self.model = load_detector()
		self.depth = None
		self.last_dist = None
		self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
		self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_cb, 10)
		self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
		self.state = 'HUNT'
		self.done = False
		self.last_dir = 1.0
		self.search_speed = SEARCH_START
		self.search_angle = 0.0
		self.search_t = None
		self.get_logger().info(f'Fetch: looking for "{TARGET}"')
		
	def flush_stop(self, n=5):
		for _ in range(n):
			self.cmd.publish(Twist())
			time.sleep(0.05)
	
	def reset_search(self):
		self.search_speed = SEARCH_START
		self.search_angle = 0.0
		self.search_t = None
		
	def wait_for_base(self, timeout=5.0):
		start = time.time()
		while self.cmd.get_subscription_count() == 0 and time.time() - start < timeout:
			time.sleep(0.1)
		if self.cmd.get_subscription_count() == 0:
			self.get_logger().warn('No /cmd_vel subscriber -- is the base controller running? Nothing will move til it is')
			
	def depth_cb(self, msg):
		d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
		self.depth = d / 1000.0
		
	def sample_depth(self, cx, cy):
		if self.depth is None:
			return None
		h, w = self.depth.shape[:2]
		cx = int(min(max(cx, 0), w - 1))
		cy = int(min(max(cy, 0), h - 1))
		x0, x1 = max(0, cx - PATCH), min(w, cx + PATCH + 1)
		y0, y1 = max(0, cy - PATCH), min(h, cy + PATCH + 1)
		patch = self.depth[y0:y1, x0:x1]
		valid = patch[(patch > 0.05) & np.isfinite(patch)]
		if valid.size == 0:
			return None
		return float(np.median(valid))
			
	def stop_and_grasp(self):
		self.state = 'GRASP'
		if self.last_dist is not None:
			remaining = max(0.0, self.last_dist - GRAB_OFFSET)
			self.get_logger().info(f'Last depth-to-bottle {self.last_dist:.2f}m '
			f'-> driving {remaining:.2f}m to close in.')
		else:
			remaining = DEFAULT_REMAIN
			self.get_logger().warn(f'No depth lock -- using fallback {remaining:.2f}m.')
		self.flush_stop()
		drive_time = remaining / CREEP_SPEED
		t = Twist()
		t.linear.x = CREEP_SPEED
		end = time.time() + drive_time
		while time.time() < end:
			self.cmd.publish(t)
			time.sleep(0.05)
		self.flush_stop()
		self.get_logger().info('Running grasp_sequence.py now!')
		subprocess.run(['python3', GRASP_SCRIPT])
		self.get_logger().info('Pickup complete.')
		self.done = True
		
	def cb(self, msg):
		if self.state != 'HUNT':
			return
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		h, w = frame.shape[:2]
		results = detect(self.model, frame)
		box = find_target_box(results, TARGET)
		twist = Twist()
		if box is not None:
			x1, y1, x2, y2 = box
			bcx, bcy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
			error = (w / 2.0) - bcx
			self.last_dir = 1.0 if error > 0 else -1.0
			self.reset_search()
			d = self.sample_depth(bcx, bcy)
			if d:
				self.last_dist = d
			if (y2 >= 0.72 * h or (self.last_dist and self.last_dist <= 0.70)) and abs(error) <= CENTER_TOL:
				self.stop_and_grasp()
				return
			twist.angular.z = turn(error, kp=KP, max_turn=MAX_TURN)
			twist.linear.x = FWD_SPEED if abs(error) <= ALIGN_TOL else 0.0
			self.cmd.publish(twist)
			
			aim = 'AIMED' if abs(error) <= ALIGN_TOL else 'turning'
			dtxt = f'{self.last_dist:.2f}m' if self.last_dist else 'no-depth'
			self.get_logger().info(f'{TARGET}: err {error:+.0f}px bottom {y2:.0f}/{h} [{aim}] '
			f'-> turn {twist.angular.z:+.2f} fwd {twist.linear.x:.2f}')
		else:
			if self.last_dist and self.last_dist <= 0.90:
				self.stop_and_grasp()
				return
			if self.search_angle < SEARCH_LIMIT:
				now = time.time()
				dt = 0.0 if self.search_t is None else now - self.search_t
				self.search_t = now
				self.search_speed = min(SEARCH_MAX, self.search_speed + SEARCH_ACCEL * dt)
				self.search_angle += self.search_speed * dt
				twist.angular.z = self.search_speed * self.last_dir
				self.cmd.publish(twist)
				self.get_logger().info(f'searching {self.last_dir:+.0f} spd {self.search_speed:.2f} '
				f'swept {self.search_angle:.1f}/{SEARCH_LIMIT:.1f} rad')
			else:
				self.get_logger().info(f'"{TARGET}" not found after a full sweep. Stopping.')
				self.flush_stop(3)
				self.done = True
				
def main():
	rclpy.init()
	node = FetchNode()
	node.wait_for_base()
	try:
		while rclpy.ok() and not node.done:
			rclpy.spin_once(node, timeout_sec=0.1)
	except KeyboardInterrupt:
		pass
	finally:
		for _ in range(5):
			node.cmd.publish(Twist())
			time.sleep(0.05)
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
			
if __name__ == '__main__':
	main()
