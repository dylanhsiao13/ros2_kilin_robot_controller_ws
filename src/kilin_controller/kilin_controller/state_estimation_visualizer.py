import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class StateVisualization(Node):
    def __init__(self):
        super().__init__('state_visualization')

        # =========================
        # Base state
        # =========================
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # =========================
        # Base state Error
        # =========================
        self.e_roll = 0.0
        self.e_pitch = 0.0
        
        self.roll_from_controller = 0.0
        self.pitch_from_controller = 0.0

        self.desired_roll = 0.0
        self.desired_pitch = 0.0    

        # =========================
        # Contact state
        # =========================
        self.contact = {'FL': False, 'FR': False, 'RL': False, 'RR': False}

        # =========================
        # Anti-fall dz command
        # =========================
        self.dz = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.dz_vis_scale = 5.0   # 視覺放大倍率

        # =========================
        # Joint states
        # =========================
        self.hip_joints = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip']
        self.joint_angles = {j: 0.0 for j in self.hip_joints}

        # =========================
        # Joint command ref (for debugging)
        # =========================
        self.joint_cmds = {j: 0.0 for j in self.hip_joints}

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(Float64MultiArray, '/base_rpy', self.rpy_cb, 10)
        self.create_subscription(Float64MultiArray, '/contact_state', self.contact_cb, 10)
        self.create_subscription(Float64MultiArray, '/dz_cmd', self.dz_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(JointState, '/joint_command_ref', self.joint_command_cb, 10)
        self.create_subscription(Float64MultiArray,'/base_attitude_error',self.error_cb,10)
        self.create_subscription(Float64MultiArray,'/com_projection',self.com_proj_cb,10)
        self.get_logger().info("StateVisualization started")

        # =========================
        # Matplotlib 3D
        # =========================
        plt.ion()
        self.fig = plt.figure(figsize=(9, 7))
        self.ax3d = self.fig.add_subplot(111, projection='3d')

        self.timer = self.create_timer(0.05, self.update_visualization)  # 20 Hz

        # =========================
        # COM
        # =========================
        self.com = np.array([0.0,0.0,0.0])
        self.com_proj = None

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def rpy_cb(self, msg: Float64MultiArray):
        self.roll = msg.data[0]
        self.pitch = msg.data[1]
        self.yaw = msg.data[2]

    def contact_cb(self, msg: Float64MultiArray):
        self.contact['FL'] = msg.data[0] > 0.5
        self.contact['FR'] = msg.data[1] > 0.5
        self.contact['RL'] = msg.data[2] > 0.5
        self.contact['RR'] = msg.data[3] > 0.5

    def dz_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            return
        self.dz['FL'] = msg.data[0]
        self.dz['FR'] = msg.data[1]
        self.dz['RL'] = msg.data[2]
        self.dz['RR'] = msg.data[3]

    def joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.hip_joints:
                self.joint_angles[name] = pos

    def joint_command_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.hip_joints:
                self.joint_cmds[name] = pos

    def error_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        self.desired_roll = msg.data[0]
        self.roll_from_controller = msg.data[1]
        self.e_roll = msg.data[2]

        self.desired_pitch = msg.data[3]
        self.pitch_from_controller = msg.data[4]
        self.e_pitch = msg.data[5]

    def com_proj_cb(self, msg: Float64MultiArray):
        # 接收來自 state_estimator 計算的 COM
        if len(msg.data) >= 3:
            self.com = np.array([msg.data[0], msg.data[1], msg.data[2]])

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def update_visualization(self):
        self.ax3d.cla()
        self.ax3d.set_xlim(-0.5, 0.5)
        self.ax3d.set_ylim(-0.5, 0.5)
        self.ax3d.set_zlim(-0.4, 0.4)
        self.ax3d.set_title(f"Anti-Fall Visualization\nroll={self.roll:.2f}, pitch={self.pitch:.2f}\n" f"error_roll={self.e_roll:.2f}, error_pitch={self.e_pitch:.2f}\n" f"desired_roll={self.desired_roll:.2f}, desired_pitch={self.desired_pitch:.2f}\n" f"controller_roll={self.roll_from_controller:.2f}, controller_pitch={self.pitch_from_controller:.2f}")

        R = self.rpy_to_rot(self.roll, self.pitch, self.yaw)

        # -------------------------
        # Body
        # -------------------------
        body_size = np.array([0.3, 0.2, 0.1])
        body_vertices = [R @ v for v in self.make_box(body_size)]
        faces = [
            [body_vertices[i] for i in idx]
            for idx in ((0, 1, 2, 3), (4, 5, 6, 7),
                        (0, 1, 5, 4), (2, 3, 7, 6),
                        (1, 2, 6, 5), (0, 3, 7, 4))
        ]
        self.ax3d.add_collection3d(Poly3DCollection(faces, alpha=0.3, facecolor='blue'))

        # -------------------------
        # Legs + dz arrow + command number
        # -------------------------
        hip_pos = {
            'FL': np.array([0.15,  0.10, 0.0]),
            'FR': np.array([0.15, -0.10, 0.0]),
            'RL': np.array([-0.15,  0.10, 0.0]),
            'RR': np.array([-0.15, -0.10, 0.0]),
        }

        leg_len = 0.25
        stance_points = []

        for leg, p_body in hip_pos.items():
            angle = -self.joint_angles[f"{leg}_hip"]
            p0 = R @ p_body
            p1 = p0 + R @ np.array([math.sin(angle), 0.0, -math.cos(angle)]) * leg_len

            # 腿線
            self.ax3d.plot([p0[0], p1[0]],
                           [p0[1], p1[1]],
                           [p0[2], p1[2]],
                           color='k', linewidth=2)

            # contact marker
            if self.contact[leg]:
                self.ax3d.scatter(p1[0], p1[1], p1[2], color='k', s=35)
                stance_points.append(p1)

            # dz arrow
            dz = self.dz[leg]
            dz_vis = dz * self.dz_vis_scale
            if abs(dz_vis) > 1e-4:
                self.ax3d.plot([p0[0], p0[0]],
                               [p0[1], p0[1]],
                               [p0[2], p0[2]+dz_vis],
                               color='red' if dz>0 else 'blue',
                               linewidth=5)

            # 顯示 joint_command_ref 數字
            hip_cmd = self.joint_cmds[f"{leg}_hip"]
            self.ax3d.text(p0[0], p0[1], p0[2]+0.03,
                           f"{hip_cmd:.2f}",
                           color='magenta', fontsize=10, fontweight='bold')

        # -------------------------
        # COM + projection to stance plane
        # -------------------------
        if len(stance_points) >= 3:
            p1, p2, p3 = stance_points[:3]
            n = np.cross(p2 - p1, p3 - p1)
            norm_n = np.linalg.norm(n)
            if norm_n > 1e-6:
                n = n / norm_n
                # COM 投影到支撐平面
                distance = np.dot(n, self.com - p1)
                self.com_proj = self.com - distance * n

                # 畫 COM
                self.ax3d.scatter(self.com[0], self.com[1], self.com[2], s=80, color='green')

                # 畫投影點
                self.ax3d.scatter(self.com_proj[0], self.com_proj[1], self.com_proj[2], s=80, color='red')

                # 畫垂直線
                self.ax3d.plot([self.com[0], self.com_proj[0]],
                               [self.com[1], self.com_proj[1]],
                               [self.com[2], self.com_proj[2]],
                               linestyle='--', color='orange')

                # 畫支撐平面
                poly = Poly3DCollection([stance_points], alpha=0.1, facecolor='cyan')
                self.ax3d.add_collection3d(poly)

        self.ax3d.set_box_aspect([1, 1, 1])
        plt.draw()
        plt.pause(0.001)

    # ------------------------------------------------------------------
    @staticmethod
    def rpy_to_rot(r, p, y):
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(r), -math.sin(r)],
                       [0, math.sin(r),  math.cos(r)]])
        Ry = np.array([[math.cos(p), 0, math.sin(p)],
                       [0, 1, 0],
                       [-math.sin(p), 0, math.cos(p)]])
        Rz = np.array([[math.cos(y), -math.sin(y), 0],
                       [math.sin(y),  math.cos(y), 0],
                       [0, 0, 1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def make_box(size):
        x, y, z = size / 2.0
        return [
            np.array([ x,  y,  z]),
            np.array([-x,  y,  z]),
            np.array([-x, -y,  z]),
            np.array([ x, -y,  z]),
            np.array([ x,  y, -z]),
            np.array([-x,  y, -z]),
            np.array([-x, -y, -z]),
            np.array([ x, -y, -z]),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = StateVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()