import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import time

from my_detect import load_detector, detect
from track_node import TARGET, DEADBAND, KP, MAX_TURN, find_largest_box, turn

STOP_DIST = 1.0
SLOW_DIST = 2.00
ALIGN_TOL = 120
FWD_KP = 0.35
MAX_FWD = 1.0
PATCH = 5

TURN_REF_DIST = 1.5
TURN_GAIN_MAX = 3.5

COAST_TIME = 0.8
SEARCH_START = 0.5
SEARCH_ACCEL = 0.6
SEARCH_MAX = 1.2
SEARCH_LIMIT = 6.28

def forward_for(dist, error):
	if not dist or dist <= STOP_DIST:
		return 0.0
	if abs(error) >= ALIGN_TOL:
		return 0.0
	speed = FWD_KP * (dist - STOP_DIST)
	return max(0.0, min(MAX_FWD, speed))
	
def turn_gain(dist):
	if not dist or dist <= 0:
		return 1.0
	return max(0.7, min(TURN_GAIN_MAX, TURN_REF_DIST / dist))
	
class ApproachObject(Node):
	def __init__(self):
		super().__init__('approach_object')
		self.bridge = CvBridge()
		self.model = load_detector()
		self.depth = None
		self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
		self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_cb, 10)
		self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
		self.last_cb = time.time()
		self.last_seen = 0.0
		self.last_dir = 0.0
		self.search_speed = SEARCH_START
		self.search_angle = 0.0
		self.search_t = None
		self.watchdog = self.create_timer(0.1, self.watchdog_cb)
		self.get_logger().info(f'Approaching "{TARGET}"')
		self.miss_count = 0
		self.MISS_THRESHOLD = 8
	
	def stop(self):
		self.cmd.publish(Twist())
		
	def reset_search(self):
		self.search_speed = SEARCH_START
		self.search_angle = 0.0
		self.search_t = None
		
	def watchdog_cb(self):
		if time.time() - self.last_cb > (SEARCH_LIMIT / SEARCH_START) + 2.0:
			self.cmd.publish(Twist())
		
	def depth_cb(self, msg):
		d = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough').astype(np.float32)
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
		
	def cb(self, msg):
		self.last_cb = time.time()
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		results = detect(self.model, frame)
		cx_img = frame.shape[1] / 2.0
		
		box = find_largest_box(results)
		twist = Twist()
		if box is not None:
			self.miss_count = 0	
			
			x1, y1, x2, y2 = box
			bcx, bcy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
			error = cx_img - bcx
			dist = self.sample_depth(bcx, bcy)
			
			self.last_seen = time.time()
			self.last_dir = 1.0 if error > 0 else -1.0
			self.reset_search()
			
			g = turn_gain(dist)
			twist.angular.z = turn(error, kp=KP * g, max_turn=MAX_TURN * g)
			twist.linear.x = forward_for(dist, error)
			
			aim = 'AIMED' if abs(error) <= DEADBAND else 'turning'
			dtxt = f'{dist:.2f}m' if dist else 'no-depth'
			near = 'ARRIVED-stop' if (dist and dist <= STOP_DIST) else 'far'
			self.get_logger().info(f'{TARGET}: err {error:+.0f}px dist {dtxt} [{aim}/{near}] ' f'-> turn {twist.angular.z:+.2f} fwd {twist.linear.x:.2f}')
			self.cmd.publish(twist)
		else:
			self.miss_count += 1
			if self.miss_count < self.MISS_THRESHOLD:
				self.cmd.publish(Twist())
				return
			if time.time() - self.last_seen < COAST_TIME and self.last_dir:
				twist.angular.z = MAX_TURN * self.last_dir
				self.cmd.publish(twist)
				self.reset_search()
			elif self.last_dir and self.search_angle < SEARCH_LIMIT:
				now = time.time()
				dt = 0.0 if self.search_t is None else now - self.search_t
				self.search_t = now
				self.search_speed = min(SEARCH_MAX, self.search_speed + SEARCH_ACCEL * dt)
				self.search_angle += self.search_speed * dt
				twist.angular.z = self.search_speed * self.last_dir
				self.cmd.publish(twist)
				self.get_logger().info(f'{TARGET} lost -> searching {self.last_dir:+.0f} ' f'spd {self.search_speed:.2f} swept {self.search_angle:.1f}/{SEARCH_LIMIT:.1f} rad')
			else:
				self.cmd.publish(Twist())
				
def main():
	rclpy.init()
	node = ApproachObject()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		for _ in range(5):
			node.stop()
			time.sleep(0.05)
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == '__main__':
	main()
