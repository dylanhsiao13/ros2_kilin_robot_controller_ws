import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import numpy as np


class HipJointTrajectoryPlanner(Node):

    def __init__(self):
        super().__init__('hip_joint_trajectory_planner')

        # =========================
        # Joint config
        # =========================
        self.hip_joints = [
            'FL_hip',
            'FR_hip',
            'RL_hip',
            'RR_hip'
        ]

        self.leg_map = {
            'FL': 'FL_hip',
            'FR': 'FR_hip',
            'RL': 'RL_hip',
            'RR': 'RR_hip'
        }

        # =========================
        # Parameters
        # =========================
        self.leg_length = 0.35    # effective leg length (m)
        self.kp_dz = 10          # dz -> hip gain

        # safety (rad)
        self.hip_min = -math.pi
        self.hip_max =  math.pi
        # =========================
        # State
        # =========================
        self.curr_hip = {j: 0.0 for j in self.hip_joints}
        self.dz_cmd = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/dz_cmd',
            self.dz_cmd_cb,
            10
        )

        # =========================
        # Publisher
        # =========================
        self.pub_cmd = self.create_publisher(
            JointState,
            '/joint_command_ref',
            10
        )

        # control loop
        self.timer = self.create_timer(0.1, self.update_cmd)  # 10 Hz

        self.get_logger().info('Hip Joint Trajectory Planner started')

    # ======================================================
    # callbacks
    # ======================================================
    def joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.curr_hip:
                self.curr_hip[name] = pos

    def dz_cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('dz_cmd size < 4')
            return

        self.dz_cmd['FL'] = msg.data[0]
        self.dz_cmd['FR'] = msg.data[1]
        self.dz_cmd['RL'] = msg.data[2]
        self.dz_cmd['RR'] = msg.data[3]

    # ======================================================
    # main logic
    # ======================================================
    def update_cmd(self):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()

        self.hip_sign = {
                'FL':  1.0,
                'FR': 1.0,
                'RL': -1.0,
                'RR': -1.0
            }
        for leg, hip_joint in self.leg_map.items():
            q_curr = self.curr_hip[hip_joint]
            dz = self.dz_cmd[leg]

            if dz > 0.0 and abs(q_curr) < 0.2:
                q_cmd = q_curr

            else:
                dtheta = self.kp_dz * dz

                sign = self.hip_sign[leg]

                q_cmd = q_curr + sign * dtheta
                

            q_cmd = float(np.clip(q_cmd, self.hip_min, self.hip_max))

            cmd.name.append(hip_joint)
            cmd.position.append(q_cmd)

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HipJointTrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
