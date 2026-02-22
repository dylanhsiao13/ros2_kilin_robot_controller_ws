import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class stair_climbing_gait_generator(Node):

    def __init__(self):
        super().__init__('stair_climbing_gait_generator')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_command_ref',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.time_start = time.time()

        # Hip order
        self.hip_joints = ['RL_hip', 'FL_hip', 'RR_hip', 'FR_hip']

        self.last_cmd_positions = [0.0] * 4

        # ===== Parameters =====
        self.declare_parameter('gait_frequency', 0.1)  # 0.1Hz
        self.declare_parameter('leg_interval', 1)    # T (sec between legs)

        self.get_logger().info(
            'Sequential Continuous Stair Climbing Gait Started'
        )

    # -------------------------
    # Utilities
    # -------------------------

    def wrap_to_pi(self, angle):
        return ((angle + math.pi) % (2.0 * math.pi)) - math.pi

    def make_continuous(self, prev, curr):
        while curr - prev > math.pi:
            curr -= 2.0 * math.pi
        while curr - prev < -math.pi:
            curr += 2.0 * math.pi
        return curr

    # -------------------------
    # Main loop
    # -------------------------

    def timer_callback(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hip_joints

        t = time.time() - self.time_start

        freq = max(self.get_parameter('gait_frequency').value, 1e-3)
        T = self.get_parameter('leg_interval').value

        P = 1.0 / freq           # one leg rotation duration
        omega = 2.0 * math.pi / P

        # New total cycle
        T_cycle = 4 * P + 4 * T

        t_mod = t % T_cycle
        n_cycles = int(t // T_cycle)

        positions = []

        for i in range(4):

            start = i * (P + T)
            end = start + P

            base = n_cycles * 2.0 * math.pi

            if t_mod < start:
                pos = base
            elif start <= t_mod < end:
                pos = base + omega * (t_mod - start)
            else:
                pos = base + 2.0 * math.pi

            # wrap + continuity
            pos_wrapped = self.wrap_to_pi(pos)

            pos_cont = self.make_continuous(
                self.last_cmd_positions[i],
                pos_wrapped
            )

            positions.append(pos_cont)
            self.last_cmd_positions[i] = pos_cont

        msg.position = positions
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = stair_climbing_gait_generator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()