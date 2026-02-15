import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import math


class BaseAttitudeGUI(Node):

    def __init__(self):
        super().__init__('base_attitude_gui')

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/base_attitude_cmd',
            10
        )

        # ------------------------
        # State (實際 publish 的值)
        # ------------------------
        self.desired_roll = 0.0     # rad
        self.desired_pitch = 0.0    # rad
        self.kp_roll = 0.05
        self.kp_pitch = 0.05
        self.enable = 1.0

        # ------------------------
        # GUI
        # ------------------------
        self.root = tk.Tk()
        self.root.title("Base Attitude GUI")

        self.roll_value_label = None
        self.pitch_value_label = None
        self.kp_roll_value_label = None
        self.kp_pitch_value_label = None

        self._build_angle_slider(
            "Desired Roll",
            -5, 5, 0,
            lambda v: self._update_roll(v)
        )

        self._build_angle_slider(
            "Desired Pitch",
            -5, 5, 1,
            lambda v: self._update_pitch(v)
        )

        self._build_gain_slider(
            "KP Roll",
            0.0, 0.2, 2,
            lambda v: self._update_kp_roll(v)
        )

        self._build_gain_slider(
            "KP Pitch",
            0.0, 0.2, 3,
            lambda v: self._update_kp_pitch(v)
        )

        self.enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            self.root,
            text="Enable Controller",
            variable=self.enable_var,
            command=self._toggle_enable
        ).grid(row=4, column=0, columnspan=3, pady=8)

        # 啟動主 loop
        self.root.after(50, self.loop)

    # --------------------------------------------------
    # Slider builders
    # --------------------------------------------------
    def _build_angle_slider(self, label, minv, maxv, row, callback):
        ttk.Label(self.root, text=f"{label} (deg)").grid(row=row, column=0, sticky="w")

        var = tk.DoubleVar()
        slider = ttk.Scale(
            self.root,
            from_=minv,
            to=maxv,
            variable=var,
            command=lambda _: callback(var.get())
        )
        slider.grid(row=row, column=1, sticky="ew", padx=10)

        value_label = ttk.Label(self.root, text="0.00 deg (0.000 rad)")
        value_label.grid(row=row, column=2, sticky="w")

        if "Roll" in label:
            self.roll_value_label = value_label
        else:
            self.pitch_value_label = value_label

    def _build_gain_slider(self, label, minv, maxv, row, callback):
        ttk.Label(self.root, text=label).grid(row=row, column=0, sticky="w")

        var = tk.DoubleVar(value=minv)
        slider = ttk.Scale(
            self.root,
            from_=minv,
            to=maxv,
            variable=var,
            command=lambda _: callback(var.get())
        )
        slider.grid(row=row, column=1, sticky="ew", padx=10)

        value_label = ttk.Label(self.root, text=f"{minv:.3f}")
        value_label.grid(row=row, column=2, sticky="w")

        if "Roll" in label:
            self.kp_roll_value_label = value_label
        else:
            self.kp_pitch_value_label = value_label

    # --------------------------------------------------
    # Update callbacks
    # --------------------------------------------------
    def _update_roll(self, deg):
        self.desired_roll = math.radians(deg)
        self.roll_value_label.config(
            text=f"{deg:6.2f} deg ({self.desired_roll: .3f} rad)"
        )

    def _update_pitch(self, deg):
        self.desired_pitch = math.radians(deg)
        self.pitch_value_label.config(
            text=f"{deg:6.2f} deg ({self.desired_pitch: .3f} rad)"
        )

    def _update_kp_roll(self, v):
        self.kp_roll = v
        self.kp_roll_value_label.config(text=f"{v:.4f}")

    def _update_kp_pitch(self, v):
        self.kp_pitch = v
        self.kp_pitch_value_label.config(text=f"{v:.4f}")

    def _toggle_enable(self):
        self.enable = 1.0 if self.enable_var.get() else 0.0

    # --------------------------------------------------
    def loop(self):
        """GUI + ROS 主循環"""
        msg = Float64MultiArray()
        msg.data = [
            self.desired_roll,
            self.desired_pitch,
            self.kp_roll,
            self.kp_pitch,
            self.enable
        ]
        self.pub.publish(msg)

        rclpy.spin_once(self, timeout_sec=0.0)

        self.root.update_idletasks()
        self.root.update()

        self.root.after(50, self.loop)


def main():
    rclpy.init()
    gui = BaseAttitudeGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
