import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class WholeBodyController(Node):
    def __init__(self):
        super().__init__('whole_body_controller')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_command_ref',
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.time_start = time.time()

        # Amble order: RL → FL → RR → FR
        self.hip_joints = ['RL_hip', 'FL_hip', 'RR_hip', 'FR_hip']

        # Track cumulative positions for each joint (continuously increasing)
        self.cumulative_positions = [0.0] * len(self.hip_joints)

        # Parameters
        self.declare_parameter('gait_frequency', 0.25)  # Hz

        self.get_logger().info(
            'WholeBodyController started: cyclic amble (continuously increasing hip motion)'
        )

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hip_joints

        t = time.time() - self.time_start

        freq = self.get_parameter('gait_frequency').value
        if freq <= 0.0:
            freq = 1.0

        period = 1.0 / freq
        omega = 2.0 * math.pi / period
        T_cycle = len(self.hip_joints) * period

        t_mod = t % T_cycle
        n_cycles = int(t // T_cycle)

        positions = []
        for i, joint in enumerate(self.hip_joints):
            start = i * period
            end = (i + 1) * period

            # Base position for completed cycles
            base_position = n_cycles * 2.0 * math.pi

            if t_mod < start:
                pos = base_position
            elif start <= t_mod < end:
                pos = base_position + omega * (t_mod - start)
            else:
                pos = base_position + 2.0 * math.pi

            # Wrap to [-pi, pi] to satisfy PhysX revolute joint limits
            pos_wrapped = ((pos + math.pi) % (2.0 * math.pi)) - math.pi
            positions.append(pos_wrapped)

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
