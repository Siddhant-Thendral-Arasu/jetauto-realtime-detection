import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from my_detect import load_detector, detect   # Imports from my_detect.py

# --- Tracking Config ---
TARGET   = 'person'
KP       = 0.003     # rad/s per pixel of horizontal error
MAX_TURN = 0.6       # cap angular speed (rad/s)
DEADBAND = 70        # px; within this of center = "aimed", don't turn


# Reusable Behavior Helpers (Importable by other nodes)
def find_largest(results, target=TARGET):
    """Return (x1, y1, x2, y2) of the largest box whose class == target, else None."""
    names = results[0].names
    best_box, best_area = None, 0.0
    for b in results[0].boxes:
        if names[int(b.cls)] == target:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area, best_box = area, (x1, y1, x2, y2)
    return best_box


def turn_for(error, kp=KP, max_turn=MAX_TURN, deadband=DEADBAND):
    """Proportional turn-to-center: horizontal pixel error -> angular.z (rad/s).
    0 inside the deadband. +error (object left) -> +z (turn left)."""
    if abs(error) <= deadband:
        return 0.0
    return max(-max_turn, min(max_turn, kp * error))


class TrackObject(Node):
    def __init__(self):
        super().__init__('track_object')
        self.bridge = CvBridge()
        self.model = load_detector()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 10)
        self.get_logger().info(f'Tracking "{TARGET}" -- turning to center it')

    def stop(self):
        self.cmd.publish(Twist())

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = detect(self.model, frame)
        cx_img = frame.shape[1] / 2.0

        box = find_largest(results)
        twist = Twist()
        if box is not None:
            x1, y1, x2, y2 = box
            error = cx_img - (x1 + x2) / 2.0
            twist.angular.z = turn_for(error)
            if twist.angular.z:
                self.get_logger().info(f'{TARGET} err {error:+.0f}px -> turn {twist.angular.z:+.2f}')
        self.cmd.publish(twist)


def main():
    rclpy.init()
    node = TrackObject()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
