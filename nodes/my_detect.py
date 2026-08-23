import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

ENGINE = '/home/ubuntu/ros2_ws/src/example/example/yolo_detect/models/26/yolo26n.engine'
CONF = 0.4


# Reusable Detection "Processes" (Importable by other nodes)
def load_detector(engine=ENGINE):
    """Load the TensorRT YOLO engine (offline, no CLIP)."""
    return YOLO(engine, task='detect')


def detect(model, frame, conf=CONF):
    """Run inference on one BGR frame. Returns the ultralytics results object."""
    return model.predict(frame, conf=conf, verbose=False)


class MyDetect(Node):
    def __init__(self):
        super().__init__('my_detect')
        self.bridge = CvBridge()
        self.model = load_detector()               # <- was: YOLO(ENGINE, task='detect')
        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
        self.get_logger().info('My detection node started (TensorRT engine)')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = detect(self.model, frame)        # <- was: self.model.predict(...)
        cv2.imwrite('/home/ubuntu/my_detected.jpg', results[0].plot())
        names = results[0].names
        labels = [names[int(b.cls)] for b in results[0].boxes]
        if labels:
            self.get_logger().info(f'Detected: {labels}')


def main():
    rclpy.init()
    node = MyDetect()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
