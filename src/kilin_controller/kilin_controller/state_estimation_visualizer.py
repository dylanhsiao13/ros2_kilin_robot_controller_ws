import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class StateVisualization(Node):
    def __init__(self):
        super().__init__('state_visualization')

        # =========================
        # Base RPY
        # =========================
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # =========================
        # Contact
        # =========================
        self.contact = {'FL': False, 'FR': False, 'RL': False, 'RR': False}

        # =========================
        # Joint states
        # =========================
        self.hip_joints = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip']
        self.susp_joints = ['FL_suspension', 'FR_suspension', 'RL_suspension', 'RR_suspension']

        self.joint_angles = {j: 0.0 for j in self.hip_joints}
        self.suspensions = {j: 0.0 for j in self.susp_joints}

        # Buffers for plotting
        self.buffer_size = 300
        self.susp_buffers = {j: deque(maxlen=self.buffer_size) for j in self.susp_joints}
        self.time_buf = deque(maxlen=self.buffer_size)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(Float64MultiArray, '/base_rpy', self.rpy_cb, 10)
        self.create_subscription(Float64MultiArray, '/contact_state', self.contact_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.get_logger().info("StateVisualization started")

        # =========================
        # Matplotlib 3D GUI
        # =========================
        plt.ion()
        self.fig = plt.figure(figsize=(10,8))
        self.ax3d = self.fig.add_subplot(211, projection='3d')
        self.ax_susp = self.fig.add_subplot(223)

        # setup lines for plotting
        self.susp_lines = {j: self.ax_susp.plot([], [], label=j)[0] for j in self.susp_joints}
        self.ax_susp.set_title("Suspension Travel")
        self.ax_susp.set_ylabel("m")
        self.ax_susp.set_xlabel("Time [s]")
        self.ax_susp.legend()
        self.ax_susp.grid(True)

        # timer
        self.timer = self.create_timer(0.05, self.update_visualization)  # 20Hz

    # ----------------------
    # Callbacks
    # ----------------------
    def rpy_cb(self, msg: Float64MultiArray):
        self.roll = msg.data[0]
        self.pitch = msg.data[1]
        self.yaw = msg.data[2]

    def contact_cb(self, msg: Float64MultiArray):
        self.contact['FL'] = msg.data[0] > 0.5
        self.contact['FR'] = msg.data[1] > 0.5
        self.contact['RL'] = msg.data[2] > 0.5
        self.contact['RR'] = msg.data[3] > 0.5

    def joint_cb(self, msg: JointState):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        self.time_buf.append(t)
        for name, pos in zip(msg.name, msg.position):
            if name in self.hip_joints:
                self.joint_angles[name] = pos
            if name in self.susp_joints:
                self.suspensions[name] = pos
                self.susp_buffers[name].append(pos)

    # ----------------------
    # Visualization
    # ----------------------
    def update_visualization(self):
        # ----------- 3D robot -----------
        self.ax3d.cla()
        self.ax3d.set_xlim(-0.5, 0.5)
        self.ax3d.set_ylim(-0.5, 0.5)
        self.ax3d.set_zlim(-0.5, 0.5)
        self.ax3d.set_title(f"Robot Visualization (roll={self.roll:.2f}, pitch={self.pitch:.2f})")

        # rotation matrix from RPY
        R = self.rpy_to_rot(self.roll, self.pitch, self.yaw)

        # body box
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
        self.ax3d.add_collection3d(Poly3DCollection(faces, alpha=0.3, facecolor='blue'))

        # hip positions
        hip_pos = {
            'FL': np.array([ 0.15,  0.10, 0.0]),
            'FR': np.array([ 0.15, -0.10, 0.0]),
            'RL': np.array([-0.15,  0.10, 0.0]),
            'RR': np.array([-0.15, -0.10, 0.0]),
        }
        leg_len = 0.25

        for leg, p_body in hip_pos.items():
            angle = -self.joint_angles[f"{leg}_hip"]
            susp_len = self.suspensions[f"{leg}_suspension"]
            p0 = R @ p_body
            leg_dir = np.array([np.sin(angle), 0.0, -np.cos(angle)])
            p1 = p0 + R @ ((leg_len + susp_len) * leg_dir)

            # leg line
            self.ax3d.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], linewidth=2, color='k')

            # contact
            if self.contact[leg]:
                self.ax3d.scatter(p1[0], p1[1], p1[2], color='k', s=40)

        self.ax3d.set_box_aspect([1,1,1])

        # ----------- suspension plot -----------
        for j, line in self.susp_lines.items():
            line.set_data(self.time_buf, self.susp_buffers[j])
        self.ax_susp.relim()
        self.ax_susp.autoscale_view()

        plt.draw()
        plt.pause(0.001)

    # ----------------------
    @staticmethod
    def rpy_to_rot(roll, pitch, yaw):
        Rx = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
        Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def make_box(size):
        x,y,z = size/2.0
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
