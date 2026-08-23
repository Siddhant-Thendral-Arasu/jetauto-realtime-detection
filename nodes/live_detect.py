#!/usr/bin/env python3
"""
Open-vocabulary object detection on the JetAuto Pro using YOLOE.

Unlike the fixed-vocabulary TensorRT node (my_detect.py), this detects objects
by *text prompt* — you list the class names you care about in plain language and
the model finds them without retraining. This is the more flexible / more current
approach.

Requires the `clip` library at runtime (used to embed the text prompts):
    pip install clip-anytorch --break-system-packages

Exclusively for camera and imported YOLO model testing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLOE
import cv2

MODEL = '/home/ubuntu/dev_ws/yoloe-11s-seg.pt'
CAMERA_TOPIC = '/depth_cam/rgb/image_raw'

# Open vocabulary: describe what to look for in plain language.
PROMPTS = ["person", "cup", "bottle", "chair", "laptop",
           "cell phone", "keyboard", "mouse", "book", "backpack"]


class LiveDetect(Node):
    def __init__(self):
        super().__init__('live_detect')
        self.bridge = CvBridge()
        self.model = YOLOE(MODEL)
        # set_classes turns the text prompts into embeddings (this is the step
        # that needs CLIP).
        self.model.set_classes(PROMPTS, self.model.get_text_pe(PROMPTS))
        self.sub = self.create_subscription(Image, CAMERA_TOPIC, self.cb, 10)
        self.get_logger().info('Open-vocab detection started (YOLOE)')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(frame, conf=0.25, verbose=False)
        cv2.imwrite('/home/ubuntu/live_detected.jpg', results[0].plot())
        labels = [results[0].names[int(b.cls)] for b in results[0].boxes]
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
