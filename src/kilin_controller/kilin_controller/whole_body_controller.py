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

        # Amble order
        self.hip_joints = ['RL_hip', 'FL_hip', 'RR_hip', 'FR_hip']

        # Store last commanded position (IMPORTANT)
        self.last_cmd_positions = [0.0] * len(self.hip_joints)

        self.declare_parameter('gait_frequency', 0.25)  # Hz

        self.get_logger().info(
            'WholeBodyController started (position control with wrap continuity)'
        )

    def wrap_to_pi(self, angle):
        return ((angle + math.pi) % (2.0 * math.pi)) - math.pi

    def make_continuous(self, prev, curr):
        """
        Adjust curr by ±2π so it is closest to prev
        """
        while curr - prev > math.pi:
            curr -= 2.0 * math.pi
        while curr - prev < -math.pi:
            curr += 2.0 * math.pi
        return curr

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hip_joints

        t = time.time() - self.time_start

        freq = self.get_parameter('gait_frequency').value
        freq = max(freq, 1e-3)

        period = 1.0 / freq
        omega = 2.0 * math.pi / period
        T_cycle = len(self.hip_joints) * period

        t_mod = t % T_cycle
        n_cycles = int(t // T_cycle)

        positions = []

        for i in range(len(self.hip_joints)):
            start = i * period
            end = (i + 1) * period

            base = n_cycles * 2.0 * math.pi

            if t_mod < start:
                pos = base
            elif start <= t_mod < end:
                pos = base + omega * (t_mod - start)
            else:
                pos = base + 2.0 * math.pi

            # 1) PhysX-safe wrap
            pos_wrapped = self.wrap_to_pi(pos)

            # 2) Time-continuous correction to avoid jumps
            pos_cont = self.make_continuous(
                self.last_cmd_positions[i],
                pos_wrapped
            )

            positions.append(pos_cont)
            self.last_cmd_positions[i] = pos_cont

        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
