import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import deque
import time

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')

        self.hip_joints = [
            'FL_hip',
            'FR_hip',
            'RL_hip',
            'RR_hip'
        ]

        self.sub_cmd = self.create_subscription(
            JointState, '/joint_command', self.cmd_callback, 10
        )
        self.sub_state = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10
        )

        self.t0 = time.time()

        self.cmd_time = {j: deque(maxlen=200) for j in self.hip_joints}
        self.cmd_pos  = {j: deque(maxlen=200) for j in self.hip_joints}
        self.state_time = {j: deque(maxlen=200) for j in self.hip_joints}
        self.state_pos  = {j: deque(maxlen=200) for j in self.hip_joints}

        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.lines = {}

        # === 建立固定的 line objects ===
        for joint in self.hip_joints:
            state_line, = self.ax.plot([], [], linewidth=2,
                                       label=f"{joint} state")
            cmd_line, = self.ax.plot([], [], '--', alpha=0.7,
                                     label=f"{joint} cmd")
            self.lines[joint] = (state_line, cmd_line)

        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Hip Position [rad]")
        self.ax.grid(True)
        self.ax.legend()

        self.timer = self.create_timer(0.1, self.update_plot)

    def cmd_callback(self, msg: JointState):
        t = time.time() - self.t0
        for name, pos in zip(msg.name, msg.position):
            if name in self.hip_joints:
                self.cmd_time[name].append(t)
                self.cmd_pos[name].append(pos)

    def state_callback(self, msg: JointState):
        t = time.time() - self.t0
        for name, pos in zip(msg.name, msg.position):
            if name in self.hip_joints:
                self.state_time[name].append(t)
                self.state_pos[name].append(pos)

    def update_plot(self):
        for joint in self.hip_joints:
            state_line, cmd_line = self.lines[joint]

            if len(self.state_time[joint]) > 1:
                state_line.set_data(
                    self.state_time[joint],
                    self.state_pos[joint]
                )

            if len(self.cmd_time[joint]) > 1:
                cmd_line.set_data(
                    self.cmd_time[joint],
                    self.cmd_pos[joint]
                )

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
