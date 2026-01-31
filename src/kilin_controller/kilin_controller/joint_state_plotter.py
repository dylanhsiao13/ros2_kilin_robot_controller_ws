import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import time
from collections import deque


class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

        # Joints to visualize
        self.joints_to_plot = ['RL_hip', 'FL_hip', 'RR_hip', 'FR_hip']

        # Data buffers
        self.time_window = 10.0  # seconds
        self.times = deque()
        self.data = {j: deque() for j in self.joints_to_plot}

        self.start_time = time.time()

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines = {}

        for joint in self.joints_to_plot:
            line, = self.ax.plot([], [], label=joint)
            self.lines[joint] = line

        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Joint Position [rad]')
        self.ax.set_title('Hip Joint Positions (JointState)')
        self.ax.legend()
        self.ax.grid(True)

        self.get_logger().info('JointStatePlotter started')

    def callback(self, msg: JointState):
        now = time.time() - self.start_time
        self.times.append(now)

        name_to_pos = dict(zip(msg.name, msg.position))

        for joint in self.joints_to_plot:
            self.data[joint].append(name_to_pos.get(joint, 0.0))

        # Remove old data
        while self.times and (now - self.times[0] > self.time_window):
            self.times.popleft()
            for joint in self.joints_to_plot:
                self.data[joint].popleft()

        self.update_plot()

    def update_plot(self):
        for joint in self.joints_to_plot:
            self.lines[joint].set_data(self.times, self.data[joint])

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
