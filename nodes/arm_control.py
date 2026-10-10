import sys
import time
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition


GRIPPER_OPEN = 200
GRIPPER_CLOSED = 800

POSES = {
    "home": {
        1: 500,
        2: 500,
        3: 500,
        4: 500,
        5: 500,
        10: GRIPPER_OPEN,
    },
    "ready": {
        1: 500,
        2: 725,
        3: 0,
        4: 150,
        5: 500,
        10: GRIPPER_CLOSED,
    },
}

MOVE_TIME = 1.0
TOPIC = "/ros_robot_controller/bus_servo/set_position"


class ArmControl(Node):
    def __init__(self):
        super().__init__("arm_control")
        self.pub = self.create_publisher(
            ServosPosition,
            TOPIC,
            10,
        )

    def send(self, joints, duration=MOVE_TIME):
        msg = ServosPosition()
        msg.duration = float(duration)

        msg.position = [
            ServoPosition(id=servo_id, position=position)
            for servo_id, position in joints.items()
        ]
        deadline = time.time() + 5.0
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
        	time.sleep(0.05)
        if self.pub.get_subscription_count() == 0:
        	self.get_logger().error('no subscriber -- is ros_robot_controller running?')
        	return
        for _ in range(3):
        	self.pub.publish(msg)
        	time.sleep(0.05)
        self.get_logger().info(f'Sent {joints} over {duration}s')


    def pose(self, name):
        if name not in POSES:
            self.get_logger().error(f'unknown pose "{name}". options: {list(POSES)}')
            return
        self.send(POSES[name])

    def gripper(self, position):
        self.send(
            {10: position},
            duration=0.5,
        )


def main():
    # Default to home.
    args = sys.argv[1:]
    cmd = sys.argv[1].lower() if len(sys.argv) > 1 else "home"
    rclpy.init()
    node = ArmControl()
    # Give ROS 2 time to establish the publisher.
    time.sleep(0.5)
    try:
        if cmd == "open":
            node.gripper(GRIPPER_OPEN)

        elif cmd == "close":
            node.gripper(GRIPPER_CLOSED)
            
        elif cmd == "jog" and len(args) == 3:
        	node.send({int(args[1]): int(args[2])}, duration=1.0)

        elif cmd in POSES:
            node.pose(cmd)

        else:
            node.get_logger().error(
                f'Unknown command "{cmd}".'
            )
            node.get_logger().info(
                "Available commands: home, ready, open, close"
            )
            return 1

        # Give the servo controller time to execute the movement.
        time.sleep(2.0)

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
