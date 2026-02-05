import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_command',
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_command_ref',
            self.command_callback,
            10
        )

        self.declare_parameter('joint_names', [
            "FL_hip", "FL_steering", "FL_suspension", "FL_wheel",
            "FR_hip", "FR_steering", "FR_suspension", "FR_wheel",
            "RL_hip", "RL_steering", "RL_suspension", "RL_wheel",
            "RR_hip", "RR_steering", "RR_suspension", "RR_wheel",
        ])

        self.joint_names = self.get_parameter('joint_names').value

        # Latest command buffer
        self.command_buffer = {}

        self.get_logger().info('Joint Commander ready')

    def command_callback(self, msg):
        # Store latest commands
        for name, pos in zip(msg.name, msg.position):
            self.command_buffer[name] = pos

        self.publish_joint_state()

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions = []
        for joint in self.joint_names:
            positions.append(self.command_buffer.get(joint, 0.0))

        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()  