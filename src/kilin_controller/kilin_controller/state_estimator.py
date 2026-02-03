import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')

        # =========================
        # State
        # =========================
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.hip_joints = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip']
        self.joint_angles = {j: 0.0 for j in self.hip_joints}

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.get_logger().info("StateEstimator started (robot visualization)")

        # =========================
        # Matplotlib 3D GUI
        # =========================
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.timer = self.create_timer(0.05, self.update_visualization)

    # ------------------------------------------------
    def imu_callback(self, msg: Imu):
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        self.roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        self.pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
        self.yaw = 0.0  # 先固定

    def joint_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_angles:
                self.joint_angles[name] = pos

    # ------------------------------------------------
    def update_visualization(self):
        self.ax.cla()

        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(-0.5, 0.5)
        self.ax.set_title("Robot State Estimator")

        R = self.rpy_to_rot(self.roll, self.pitch, self.yaw)

        # =========================
        # Body (box)
        # =========================
        body_size = np.array([0.3, 0.2, 0.1])
        body_vertices = self.make_box(body_size)
        body_vertices = [R @ v for v in body_vertices]

        faces = [
            [body_vertices[i] for i in idx]
            for idx in (
                (0,1,2,3), (4,5,6,7),
                (0,1,5,4), (2,3,7,6),
                (1,2,6,5), (0,3,7,4)
            )
        ]

        self.ax.add_collection3d(
            Poly3DCollection(faces, alpha=0.3)
        )

        # =========================
        # Legs (lines)
        # =========================
        hip_pos = {
            'FL_hip': np.array([ 0.15,  0.10, 0.0]),
            'FR_hip': np.array([ 0.15, -0.10, 0.0]),
            'RL_hip': np.array([-0.15,  0.10, 0.0]),
            'RR_hip': np.array([-0.15, -0.10, 0.0]),
        }

        leg_len = 0.25

        for name, p_body in hip_pos.items():
            angle = -self.joint_angles[name]

            # leg direction in body frame (pitch)
            leg_dir = np.array([
                math.sin(angle),
                0.0,
                -math.cos(angle)
            ])

            p0 = R @ p_body
            p1 = p0 + R @ (leg_len * leg_dir)

            self.ax.plot(
                [p0[0], p1[0]],
                [p0[1], p1[1]],
                [p0[2], p1[2]],
                linewidth=2
            )

        self.ax.set_box_aspect([1,1,1])
        plt.draw()
        plt.pause(0.001)

    # ------------------------------------------------
    @staticmethod
    def rpy_to_rot(r, p, y):
        Rx = np.array([
            [1,0,0],
            [0,math.cos(r),-math.sin(r)],
            [0,math.sin(r), math.cos(r)]
        ])
        Ry = np.array([
            [ math.cos(p),0,math.sin(p)],
            [0,1,0],
            [-math.sin(p),0,math.cos(p)]
        ])
        Rz = np.array([
            [math.cos(y),-math.sin(y),0],
            [math.sin(y), math.cos(y),0],
            [0,0,1]
        ])
        return Rz @ Ry @ Rx

    @staticmethod
    def make_box(size):
        x,y,z = size / 2.0
        return [
            np.array([ x, y, z]),
            np.array([-x, y, z]),
            np.array([-x,-y, z]),
            np.array([ x,-y, z]),
            np.array([ x, y,-z]),
            np.array([-x, y,-z]),
            np.array([-x,-y,-z]),
            np.array([ x,-y,-z]),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
