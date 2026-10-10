import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from my_detect import ENGINE, CONF

CAMERA_TOPIC = '/depth_cam/rgb/image_raw'
WARMUP = 15
KEEP_FRAMES = 100

def stats(ms):
	a = np.asarray(ms)
	return f'mean {a.mean():6.1f} ms | p50 {np.percentile(a, 50):6.1f} | p95 {np.percentile(a, 95):6.1f}'
	
class LiveBench(Node):
	def __init__(self, model, n_frames):
		super().__init__('detect_bench')
		self.bridge = CvBridge()
		self.model = model
		self.n_frames = n_frames
		self.seen = 0
		self.age_ms = []
		self.infer_ms = []
		self.frames = []
		self.t_first = None
		self.t_last = None
		self.done = False
		self.create_subscription(Image, CAMERA_TOPIC, self.cb, qos_profile_sensor_data)
		self.get_logger().info(f'Live phase: {n_frames} frames from {CAMERA_TOPIC}')
		
	def cb(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		t0 = time.perf_counter()
		self.model.predict(frame, conf=CONF, verbose=False)
		t1 = time.perf_counter()
		self.seen += 1
		if self.seen <= WARMUP:
			return
		if self.t_first is None:
			self.t_first = t1
		now = self.get_clock().now().nanoseconds
		stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
		if stamp > 0:
			self.age_ms.append((now - stamp) / 1e6)
		self.infer_ms.append((t1 - t0) * 1000)
		if len(self.infer_ms) % 50 == 0:
			print(f' live: {len(self.infer_ms)}/{self.n_frames} frames', flush=True)
		if len(self.frames) < KEEP_FRAMES:
			self.frames.append(frame)
		if len(self.infer_ms) >= self.n_frames:
			self.t_last = t1
			self.done = True
	
	def report(self):
		n = len(self.infer_ms)
		fps = (n - 1) / (self.t_last - self.t_first) if n > 1 else 0.0
		print('\n=== LIVE (TensorRT engine on the camera stream) ===')
		print(f'Frames processed: {n}')
		print(f'Processed FPS: {fps:.1f}')
		print(f'Inference: {stats(self.infer_ms)}')
		if self.age_ms:
			print(f'camera -> detections : {stats(self.age_ms)}')
		else:
			print('camera -> detections: n/a (camera driver leaves header stamps empty)')
		return fps
		
def offline(name, model, frames, **kw):
	for f in frames[:WARMUP]:
		model.predict(f, conf=CONF, verbose=False, **kw)
	ms = []
	for f in frames:
		t0 = time.perf_counter()
		model.predict(f, conf=CONF, verbose=False, **kw)
		ms.append((time.perf_counter() - t0) * 1000)
	a = np.asarray(ms)
	print(f'{name:<22}: {stats(ms)} | {1000 / a.mean():5.1f} FPS')
	return a.mean()
	
def main():
	ap = argparse.ArgumentParser()
	ap.add_argument('--engine', default=ENGINE)
	ap.add_argument('--pt', default=ENGINE.replace('.engine', '.pt'), help='PyTorch weights for the same model (default: next to the engine)')
	ap.add_argument('--frames', type=int, default=200)
	args, _ = ap.parse_known_args()
	
	print(f'Loading TensorRT engine: {args.engine}', flush=True)
	engine = YOLO(args.engine, task='detect')
	
	rclpy.init()
	node = LiveBench(engine, args.frames)
	try:
		while rclpy.ok() and not node.done:
			rclpy.spin_once(node, timeout_sec=0.5)
	except KeyboardInterrupt:
		print('\nStopped early.')
	if len(node.infer_ms) < 2:
		sys.exit(1)
	if node.t_last is None:
		node.t_last = time.perf_counter()
	live_fps = node.report()
	frames = node.frames
	node.destroy_node()
	rclpy.shutdown()
	
	print(f'\n=== OFFLINE (same {len(frames)} frames through both models) ===')
	trt_ms = offline('TensorRT engine', engine, frames)
	try:
		pt = YOLO(args.pt, task='detect')
		pt_ms = offline('PyTorch (.pt, GPU)', pt, frames, device=0)
	except Exception as e:
		print(f'PyTorch run skipped: {e}\n(pass -- pt with the path to the .pt weights)')
		pt_ms = None
	
	if pt_ms:
		print(f'TensorRT is {pt_ms / trt_ms:.1f}x faster than PyTorch '
		f'({trt_ms:.1f} ms vs {pt_ms:.1f} ms per frame)')
	print(f'Live: {live_fps:.0f} FPS processed on the Jetson Orin Nano')
	if node.age_ms:
		print(f'Camera-to-detection latency: {np.percentile(node.age_ms, 50):.0f} ms median')
		
if __name__ == '__main__':
	main()
		
