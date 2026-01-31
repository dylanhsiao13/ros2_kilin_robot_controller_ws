import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class WholeBodyController(Node):
    def __init__(self):
        super().__init__('whole_body_controller')

        # Publisher for joint commands
        self.publisher = self.create_publisher(
            JointState,
            '/joint_command_ref',
            10
        )

        # Timer for periodic updates
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.time_start = time.time()

        # Hip joints
        self.hip_joints = ['RL_hip', 'FL_hip', 'RR_hip', 'FR_hip']

        # Amble gait phase offsets (RL → FL → RR → FR)
        self.phase_map = {
            'RL_hip': 0.0,
            'FL_hip': math.pi / 2,
            'RR_hip': math.pi,
            'FR_hip': 3 * math.pi / 2,
        }

        # ---- PARAMETERS ----
        self.declare_parameter('gait_frequency', 1)  # Hz

        self.get_logger().info(
            'Whole Body Controller started (circular hip motion)'
        )

    def timer_callback(self):
        # Prepare joint command
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hip_joints

        t = time.time() - self.time_start

        # Get frequency parameter (can be changed live)
        freq = self.get_parameter('gait_frequency').value
        omega = 2.0 * math.pi * freq  # rad/s
        period = 1.0 / freq if freq > 0 else 1.0

        # Sequential hip rotation: RL → FL → RR → FR
        # Each joint completes one full rotation (2π) before the next starts
        positions = []
        for i, joint in enumerate(self.hip_joints):
            start_time = i * period  # Each joint starts after previous completes one cycle
            end_time = (i + 1) * period
            
            if t < start_time:
                # Not started yet, stay at 0
                positions.append(0.0)
            elif t < end_time:
                # Currently rotating
                positions.append(omega * (t - start_time))
            else:
                # Completed one full rotation (2π), stay at 2π
                positions.append(2.0 * math.pi)
        msg.position = positions

        msg.velocity = [0.0] * len(self.hip_joints)
        msg.effort = [0.0] * len(self.hip_joints)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
