import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
import math

class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')

        # IMU subscription
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.get_logger().info("StateEstimator started (Matplotlib GUI)")

        # buffers for plotting
        self.buffer_size = 300  # number of points to show
        self.time_buf = []
        self.roll_buf = []
        self.pitch_buf = []
        self.wx_buf = []
        self.wy_buf = []
        self.wz_buf = []

        # last imu
        self.last_time = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0

        # setup matplotlib
        plt.ion()
        self.fig, (self.ax_rp, self.ax_w) = plt.subplots(2,1, sharex=True)

        # roll/pitch lines
        self.line_roll, = self.ax_rp.plot([], [], label="roll")
        self.line_pitch, = self.ax_rp.plot([], [], label="pitch")
        self.ax_rp.set_ylabel("rad")
        self.ax_rp.legend()
        self.ax_rp.grid(True)

        # angular velocity lines
        self.line_wx, = self.ax_w.plot([], [], label="wx")
        self.line_wy, = self.ax_w.plot([], [], label="wy")
        self.line_wz, = self.ax_w.plot([], [], label="wz")
        self.ax_w.set_ylabel("rad/s")
        self.ax_w.set_xlabel("Time [s]")
        self.ax_w.legend()
        self.ax_w.grid(True)

        # timer for updating plot
        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.update_plot)

    def imu_callback(self, msg: Imu):
        t = self.get_clock().now().nanoseconds * 1e-9

        # quaternion -> roll/pitch
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        self.pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))

        self.wx = msg.angular_velocity.x
        self.wy = msg.angular_velocity.y
        self.wz = msg.angular_velocity.z

        # append to buffers
        self.time_buf.append(t)
        self.roll_buf.append(self.roll)
        self.pitch_buf.append(self.pitch)
        self.wx_buf.append(self.wx)
        self.wy_buf.append(self.wy)
        self.wz_buf.append(self.wz)

        # keep buffer size
        if len(self.time_buf) > self.buffer_size:
            self.time_buf.pop(0)
            self.roll_buf.pop(0)
            self.pitch_buf.pop(0)
            self.wx_buf.pop(0)
            self.wy_buf.pop(0)
            self.wz_buf.pop(0)

    def update_plot(self):
        if len(self.time_buf) < 2:
            return

        # update roll/pitch
        self.line_roll.set_data(self.time_buf, self.roll_buf)
        self.line_pitch.set_data(self.time_buf, self.pitch_buf)
        self.ax_rp.relim()
        self.ax_rp.autoscale_view()

        # update angular velocity
        self.line_wx.set_data(self.time_buf, self.wx_buf)
        self.line_wy.set_data(self.time_buf, self.wy_buf)
        self.line_wz.set_data(self.time_buf, self.wz_buf)
        self.ax_w.relim()
        self.ax_w.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


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
