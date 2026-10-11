import time
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition

OPEN, CLOSED = 200, 800

STEPS = [
	("ready", {1: 500, 2: 600, 3: 400, 4: 400, 5: 500, 10: OPEN}, 1.5),
	("reach", {1: 500, 2: 250, 3: 200, 4: 250, 5: 500, 10: OPEN}, 2.0),
	("close", {10: CLOSED},                                       1.0),
	("lift", {2: 600, 3: 400, 4: 400, 10: CLOSED},                1.5),
]
MOVE_TIME = 1.2

class Grasp(Node):
	def __init__(self):
		super().__init__('grasp_sequence')
		self.pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
		
	def send(self, joints, duration=MOVE_TIME):
		msg = ServosPosition()
		msg.duration = float(duration)
		msg.position = [ServoPosition(id=i, position=p) for i, p in joints.items()]
		deadline = time.time() + 5.0
		while self.pub.get_subscription_count() == 0 and time.time() < deadline:
			time.sleep(0.05)
		if self.pub.get_subscription_count() == 0:
			self.get_logger().error('no subscriber -- is ros_robot_controller running?')
			return False
		for _ in range(3):
			self.pub.publish(msg)
			time.sleep(0.05)
		return True
		
	def run(self):
		for name, joints, settle in STEPS:
			ok = self.send(joints)
			self.get_logger().info(f'{"OK " if ok else "FAIL"} {name}: {joints}')
			if not ok:
				self.get_logger().error('aborting grasp -- controller not connected')
				return
			time.sleep(settle)
		self.get_logger().info('grasp sequence complete')
		
def main():
	rclpy.init()
	node = Grasp()
	time.sleep(0.5)
	node.run()
	node.destroy_node()
	if rclpy.ok():
		rclpy.shutdown()
		
if __name__ == '__main__':
	main()
