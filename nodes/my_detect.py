import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

ENGINE = '/home/ubuntu/ros2_ws/src/example/example/yolo_detect/models/26/yolo26n.engine'
CONF = 0.4

def load_detector(engine=ENGINE):
	return YOLO(ENGINE, task='detect')
	
def detect(model, frame, conf=CONF):
	return model.predict(frame, conf=conf, verbose=False)

class MyDetect(Node):
	def __init__(self):
		super().__init__('my_detect')
		self.bridge = CvBridge()
		self.model = load_detector()
		self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
		self.get_logger().info('Detection Node Started (TensorRT Engine)')
	
	def cb(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		results = detect(self.model, frame)
		cv2.imwrite('/home/ubuntu/my_detected.jpg', results[0].plot())
		labels = [results[0].names[int(b.cls)] for b in results[0].boxes]
		if labels:
			self.get_logger().info(f'Detected: {labels}')

def main():
	rclpy.init()
	node = MyDetect()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
if __name__ == '__main__':
	main()
