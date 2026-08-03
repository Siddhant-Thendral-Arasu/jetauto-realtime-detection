#!/usr/bin/env python3
"""
Real-time object detection on the JetAuto Pro using a TensorRT YOLO engine.

Subscribes to the robot's depth-camera RGB stream, runs a pre-compiled TensorRT
YOLO engine on each frame (GPU inference, fully offline), logs detected classes,
and writes annotated frames to disk. Uncomment the cv2.imshow lines for a live
display window.

Written from scratch after studying the vendor's engine-loading pattern, so the
whole pipeline (camera -> ROS Image -> cv_bridge -> TensorRT inference -> labels)
is understood end to end.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

# Path to the pre-compiled TensorRT engine that ships with the platform.
# Ultralytics' YOLO() loads a .engine directly and runs GPU-accelerated inference.
ENGINE = '/home/ubuntu/ros2_ws/src/example/example/yolo_detect/models/26/yolo26n.engine'

# Camera topic (Orbbec Astra Pro Plus RGB stream).
CAMERA_TOPIC = '/depth_cam/rgb/image_raw'


class MyDetect(Node):
    def __init__(self):
        super().__init__('my_detect')
        self.bridge = CvBridge()
        self.model = YOLO(ENGINE, task='detect')  # loads TensorRT engine onto the GPU
        self.sub = self.create_subscription(Image, CAMERA_TOPIC, self.cb, 10)
        self.get_logger().info('Detection node started (TensorRT engine)')

    def cb(self, msg):
        # ROS Image -> OpenCV BGR frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # GPU inference on this frame
        results = self.model.predict(frame, conf=0.4, verbose=False)

        # Save the annotated frame (boxes + labels) for inspection
        cv2.imwrite('/home/ubuntu/my_detected.jpg', results[0].plot())

        # NOTE: class names come off the results object (works for both .engine and .pt).
        labels = [results[0].names[int(b.cls)] for b in results[0].boxes]
        if labels:
            self.get_logger().info(f'Detected: {labels}')

        # --- Live display window (uncomment for a real-time video window) ---
        # cv2.imshow('detections', results[0].plot())
        # cv2.waitKey(1)


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
