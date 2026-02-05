import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from collections import deque


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
        self.susp_joints = ['FL_suspension', 'FR_suspension', 'RL_suspension', 'RR_suspension']

        # latest joint values
        self.joint_angles = {j: 0.0 for j in self.hip_joints}
        self.suspensions = {j: 0.0 for j in self.susp_joints}
        self.hip_efforts = {j: 0.0 for j in self.hip_joints}

        # contact thresholds
        self.susp_contact_th = 0.001
        self.hip_contact_th = 20.0 

        # buffers for plotting
        self.buffer_size = 300
        self.susp_buffers = {j: deque(maxlen=self.buffer_size) for j in self.susp_joints}
        self.effort_buffers = {j: deque(maxlen=self.buffer_size) for j in self.hip_joints}
        self.time_buf = deque(maxlen=self.buffer_size)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.get_logger().info("StateEstimator started (robot visualization + plot)")

        # =========================
        # Matplotlib 3D GUI
        # =========================
        plt.ion()
        self.fig = plt.figure(figsize=(10,8))
        self.ax3d = self.fig.add_subplot(211, projection='3d')
        self.ax_susp = self.fig.add_subplot(223)
        self.ax_eff = self.fig.add_subplot(224)

        # setup lines for plotting
        self.susp_lines = {j: self.ax_susp.plot([], [], label=j)[0] for j in self.susp_joints}
        self.eff_lines = {j: self.ax_eff.plot([], [], label=j)[0] for j in self.hip_joints}

        self.ax_susp.set_title("Suspension Travel")
        self.ax_susp.set_ylabel("m")
        self.ax_susp.set_xlabel("Time [s]")
        self.ax_susp.legend()
        self.ax_susp.grid(True)

        self.ax_eff.set_title("Hip Effort")
        self.ax_eff.set_ylabel("Nm")
        self.ax_eff.set_xlabel("Time [s]")
        self.ax_eff.legend()
        self.ax_eff.grid(True)

        # timer
        self.timer = self.create_timer(0.05, self.update_visualization)  # 20Hz

    # ----------------------
    def imu_callback(self, msg: Imu):
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        self.pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
        self.yaw = 0.0  # 固定 yaw

    def joint_callback(self, msg: JointState):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        self.time_buf.append(t)

        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            if name in self.hip_joints:
                self.joint_angles[name] = pos
                self.hip_efforts[name] = eff
                self.effort_buffers[name].append(eff)
            if name in self.susp_joints:
                self.suspensions[name] = pos
                self.susp_buffers[name].append(pos)

    # ----------------------
    def update_visualization(self):
        # ----------- 3D robot -----------
        self.ax3d.cla()
        self.ax3d.set_xlim(-0.5, 0.5)
        self.ax3d.set_ylim(-0.5, 0.5)
        self.ax3d.set_zlim(-0.5, 0.5)
        self.ax3d.set_title("Robot State Estimator (3D)")

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
            'FL_hip': np.array([ 0.15,  0.10, 0.0]),
            'FR_hip': np.array([ 0.15, -0.10, 0.0]),
            'RL_hip': np.array([-0.15,  0.10, 0.0]),
            'RR_hip': np.array([-0.15, -0.10, 0.0]),
        }
        leg_len = 0.25

        for leg, p_body in hip_pos.items():
            angle = -self.joint_angles[leg]
            susp_len = self.suspensions[leg.replace('_hip','_suspension')]
            hip_eff = self.hip_efforts[leg]

            leg_dir = np.array([math.sin(angle), 0.0, -math.cos(angle)])
            p0 = R @ p_body
            p1 = p0 + R @ ((leg_len + susp_len) * leg_dir)

            # leg line
            self.ax3d.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], linewidth=2, color='k')

            # contact check: 懸空=travel<threshold or effort<threshold
            contact =  (abs(susp_len) > self.susp_contact_th or abs(hip_eff) > self.hip_contact_th)
            if contact:
                self.ax3d.scatter(p1[0], p1[1], p1[2], color='k', s=40)

        self.ax3d.set_box_aspect([1,1,1])

        # ----------- suspension plot -----------
        for j, line in self.susp_lines.items():
            line.set_data(self.time_buf, self.susp_buffers[j])
        self.ax_susp.relim()
        self.ax_susp.autoscale_view()

        # ----------- effort plot -----------
        for j, line in self.eff_lines.items():
            line.set_data(self.time_buf, self.effort_buffers[j])
        self.ax_eff.relim()
        self.ax_eff.autoscale_view()

        plt.draw()
        plt.pause(0.001)

    # ----------------------
    @staticmethod
    def rpy_to_rot(r, p, y):
        Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r), math.cos(r)]])
        Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
        Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y), math.cos(y),0],[0,0,1]])
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


if __name__ == '__main__':
    main()
