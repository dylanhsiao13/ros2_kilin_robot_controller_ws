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

        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.kp_roll = 0.05
        self.kp_pitch = 0.05
        self.enable = 1.0

        self.root = tk.Tk()
        self.root.title("Base Attitude GUI")

        self._build_slider("Desired Roll (deg)", -15, 15, 0,
                           lambda v: self._set('roll', math.radians(v)))
        self._build_slider("Desired Pitch (deg)", -15, 15, 1,
                           lambda v: self._set('pitch', math.radians(v)))
        self._build_slider("KP Roll", 0.0, 0.2, 2,
                           lambda v: self._set('kp_roll', v))
        self._build_slider("KP Pitch", 0.0, 0.2, 3,
                           lambda v: self._set('kp_pitch', v))

        self.enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            self.root, text="Enable Controller",
            variable=self.enable_var,
            command=self._toggle_enable
        ).grid(row=4, column=0, columnspan=2)

        self.timer = self.create_timer(0.05, self.publish_cmd)

    # --------------------------------------------------
    def _build_slider(self, label, minv, maxv, row, callback):
        ttk.Label(self.root, text=label).grid(row=row, column=0)
        var = tk.DoubleVar()
        slider = ttk.Scale(
            self.root, from_=minv, to=maxv,
            variable=var,
            command=lambda _: callback(var.get())
        )
        slider.grid(row=row, column=1, sticky="ew")

    def _set(self, name, value):
        setattr(self, name if name != 'roll' else 'desired_roll', value)
        setattr(self, name if name != 'pitch' else 'desired_pitch', value)

    def _toggle_enable(self):
        self.enable = 1.0 if self.enable_var.get() else 0.0

    # --------------------------------------------------
    def publish_cmd(self):
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


def main():
    rclpy.init()
    gui = BaseAttitudeGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
