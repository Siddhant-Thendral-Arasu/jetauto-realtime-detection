import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from my_detect import load_detector, detect

TARGET='bottle'
CONF = 0.25
INNER = 0.5
NEAR_PCT = 25
MIN_VALID = 20

class Locate3D(Node):
	def __init__(self):
		super().__init__('locate_3d')
		self.bridge = CvBridge()
		self.model = load_detector()
		self.depth = None
		self.K = None
		
		self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.info_cb, 10)
		self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_cb, 10)
		self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
		self.get_logger().info(f'Locate_3D: Back-Projecting "{TARGET}". Waiting for camera_info...')
		
	def info_cb(self, msg):
		if self.K is None:
			self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
			self.get_logger().info(f'intrinsics: fx={self.K[0]:.1f} fy={self.K[1]:.1f} '
			f'cx={self.K[2]:.1f} cy = {self.K[3]:.1f} ({msg.width}x{msg.height})')
			
	def depth_cb(self, msg):
		d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
		self.depth = d / 1000.0
		
	def sample_depth_box(self, x1, y1, x2, y2, rgb_w, rgb_h):
		if self.depth is None:
			return None
		dh, dw = self.depth.shape[:2]
		sx, sy = dw / float(rgb_w), dh / float(rgb_h)
		bw, bh = x2 - x1, y2 - y1
		mx, my = bw * (1 - INNER) / 2.0, bh * (1 - INNER) / 2.0
		xa = int(max(0, (x1 + mx) * sx))
		xb = int(min(dw, (x2 - mx) * sx))
		ya = int(max(0, (y1 + my) * sy))
		yb = int(min(dh, (y2 - my) * sy))
		if xb <= xa or yb <= ya:
			return None, 0
		region = self.depth[ya:yb, xa:xb]
		valid = region[(region > 0.05) & np.isfinite(region)]
		if valid.size < MIN_VALID:
			return None, int(valid.size)
		return float(np.percentile(valid, NEAR_PCT)), int(valid.size)
		
	def cb(self, msg):
		if self.K is None:
			return
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		rgb_h, rgb_w = frame.shape[:2]
		results = detect(self.model, frame, conf=CONF)
		r = results[0]
		best, best_area = None, 0.0
		for b in r.boxes:
			if r.names[int(b.cls)] != TARGET:
				continue
			x1, y1, x2, y2 = b.xyxy[0].tolist()
			a = (x2 - x1) * (y2 - y1)
			if a > best_area:
				best_area, best = a, (x1, x2, y1, y2)
		if best is None:
			return
		x1, y1, x2, y2 = best
		u = (x1 + x2) / 2.0
		v = (y1 + y2) / 2.0
		Z, n = self.sample_depth_box(x1, y1, x2, y2, rgb_w, rgb_h)
		if Z is None:
			self.get_logger().info(f'{TARGET} at px ({u:.0f},{v:.0f}) but no valid depth there. ({n} px)')
			return
		fx, fy, cx, cy = self.K
		X = (u - cx) * Z / fx
		Y = (v - cy) * Z / fy
		self.get_logger().info(f'{TARGET} at CAMERA frame: X={X:+.3f} Y={Y:+.3f} Z={Z:+.3f} m (px {u:.0f},{v:.0f})')
		
		
def main():
	rclpy.init()
	node = Locate3D()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
			
if __name__ == '__main__':
	main()
