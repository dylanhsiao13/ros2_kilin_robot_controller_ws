import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class CustomJointController(Node):
    def __init__(self):
        super().__init__('custom_joint_controller')

        # Safety limits (rad)
        self.hip_min = -math.pi  # -180 deg
        self.hip_max =  math.pi  # +180 deg

        # Latest desired joint command
        self.joint_cmd = [0.0, 0.0, 0.0, 0.0]

        # Subscribers
        self.create_subscription(Float64MultiArray, '/custom_joint_cmd', self.cmd_cb, 10)

        # Publisher
        self.pub = self.create_publisher(JointState, '/joint_command_ref', 10)

        # Timer
        self.timer = self.create_timer(0.02, self.publish_cmd)

    def cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            return
        # Clip to safety limits
        self.joint_cmd = [float(np.clip(a, self.hip_min, self.hip_max)) for a in msg.data]

    def publish_cmd(self):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['FL_hip','FR_hip','RL_hip','RR_hip']
        cmd.position = self.joint_cmd
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = CustomJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
