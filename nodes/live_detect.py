import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLOE
import cv2

class LiveDetect(Node):
	def __init__(self):
		super().__init__('live_detect')
		self.bridge = CvBridge()
		self.model = YOLOE('/home/ubuntu/dev_ws/yoloe-11s-seg.pt')
		names = ["person","cup","bottle","chair","laptop","cell phone","keyboard","mouse","book","backpack"]
		self.model.set_classes(names, self.model.get_text_pe(names))
		self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
		self.get_logger().info('Detection Started')
	
	def cb(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		results = self.model.predict(frame, conf=0.25, verbose=False)
		cv2.imwrite('/home/ubuntu/live_detected.jpg', results[0].plot())
		labels = [self.model.names[int(b.cls)] for b in results[0].boxes]
		if labels:
			self.get_logger().info(f'Detected: {labels}')

def main():
	rclpy.init()
	node = LiveDetect()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
	
