import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

# Upper controller node generating amble gait commands
class WholeBodyController(Node):
    def __init__(self):
        super().__init__('whole_body_controller')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_command_ref',
            10
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.time_start = time.time()

        # Hip joints
        self.hip_joints = ['RL_hip', 'FL_hip', 'RR_hip', 'FR_hip']

        # Amble phase offsets
        self.phase_map = {
            'RL_hip': 0.0,
            'FL_hip': math.pi / 2,
            'RR_hip': math.pi,
            'FR_hip': 3 * math.pi / 2,
        }

        # ---- PARAMETERS ----
        self.declare_parameter('gait_frequency', 0.25)  # Hz
        self.declare_parameter('amplitude', 0.5)        # rad

        self.get_logger().info(
            'Whole Body Controller started (params: gait_frequency, amplitude)'
        )

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hip_joints

        t = time.time() - self.time_start

        # Read parameters (runtime adjustable)
        freq = self.get_parameter('gait_frequency').value
        amplitude = self.get_parameter('amplitude').value

        omega = 2.0 * math.pi * freq

        msg.position = [
            amplitude * math.sin(omega * t + self.phase_map[joint])
            for joint in self.hip_joints
        ]

        self.publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyController()
    rclpy.spin(node)
    rclpy.shutdown()