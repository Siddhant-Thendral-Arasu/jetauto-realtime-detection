import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition

HOME = {1: 500, 2: 500, 3: 500, 4: 500, 5: 500}
MOVE_TIME = 2.0

class ArmHome(Node):
	def __init__(self):
		super().__init__('arm_home')
		self.pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
		self.timer = self.create_timer(0.5, self.go_home)
		self.done = False
		
	def go_home(self):
		if self.done:
			return
		msg = ServosPosition()
		msg.duration = float(MOVE_TIME)
		msg.position = [ServoPosition(id=i, position=p) for i, p in HOME.items()]
		self.pub.publish(msg)
		self.get_logger().info(f'Sent home pose: {HOME} over {MOVE_TIME}s')
		self.done = True
		
def main():
	rclpy.init()
	node = ArmHome()
	try:
		import time
		end = time.time() + 3.0
		while time.time() < end and rclpy.ok():
			rclpy.spin_once(node, timeout_sec=0.1)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
			
if __name__ == '__main__':
	main()
		
