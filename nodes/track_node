import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from my_detect import load_detector, detect

TARGET   = 'person'
KP       = 0.004
MAX_TURN = 0.6
DEADBAND = 40


class TrackObject(Node):
    def __init__(self):
        super().__init__('track_object')
        self.bridge = CvBridge()
        self.model = load_detector()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
        self.get_logger().info(f'Tracking "{TARGET}" now')

    def stop(self):
        self.cmd.publish(Twist())

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = detect(self.model, frame)
        names = results[0].names

        cx_img = frame.shape[1] / 2.0
        target_box, best_area = None, 0.0
        for b in results[0].boxes:
            if names[int(b.cls)] == TARGET:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area, target_box = area, (x1, x2)

        twist = Twist()
        if target_box is not None:
            x1, x2 = target_box
            error = cx_img - (x1 + x2) / 2.0
            if abs(error) > DEADBAND:
                twist.angular.z = max(-MAX_TURN, min(MAX_TURN, KP * error))
                self.get_logger().info(f'{TARGET} off-center {error:+.0f}px -> turn {twist.angular.z:+.2f}')
        self.cmd.publish(twist)


def main():
    rclpy.init()
    node = TrackObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
