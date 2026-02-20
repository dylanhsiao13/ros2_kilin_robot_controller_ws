import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import math


class BaseAttitudeGUI(Node):

    def __init__(self):
        super().__init__('base_attitude_gui')

        # ----------------------------------
        # Publisher
        # ----------------------------------
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/base_attitude_cmd',
            10
        )

        # ----------------------------------
        # State
        # ----------------------------------
        self.desired_roll = 0.0
        self.desired_pitch = 0.0

        self.kp_roll = 0.05
        self.kp_pitch = 0.05

        self.kd_roll = 0.01
        self.kd_pitch = 0.01

        self.ki_roll = 0.01
        self.ki_pitch = 0.01


        self.enable = 1.0

        # ----------------------------------
        # GUI
        # ----------------------------------
        self.root = tk.Tk()
        self.root.title("Base Attitude GUI")

        self.roll_value_label = None
        self.pitch_value_label = None

        self.kp_roll_value_label = None
        self.kp_pitch_value_label = None
        self.kd_roll_value_label = None
        self.kd_pitch_value_label = None
        self.ki_roll_value_label = None
        self.ki_pitch_value_label = None
        

        # Desired angles
        self._build_angle_slider(
            "Desired Roll",
            -10, 10, 0,
            self._update_roll
        )

        self._build_angle_slider(
            "Desired Pitch",
            -10, 10, 1,
            self._update_pitch
        )

        # Gains
        self._build_gain_slider(
            "KP Roll",
            0.0, 0.02, 2,
            self._update_kp_roll
        )

        self._build_gain_slider(
            "KP Pitch",
            0.0, 0.02, 3,
            self._update_kp_pitch
        )

        self._build_gain_slider(
            "KD Roll",
            0.0, 0.02, 4,
            self._update_kd_roll
        )

        self._build_gain_slider(
            "KD Pitch",
            0.0, 0.02, 5,
            self._update_kd_pitch
        )

        self._build_gain_slider(
            "KI Roll",
            0.0, 0.02, 6,
            self._update_ki_roll
        )

        self._build_gain_slider(
            "KI Pitch",
            0.0, 0.02, 7,
            self._update_ki_pitch
        )

        # Enable checkbox
        self.enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            self.root,
            text="Enable Controller",
            variable=self.enable_var,
            command=self._toggle_enable
        ).grid(row=8, column=0, columnspan=3, pady=8)

        # Start loop
        self.root.after(50, self.loop)

    # ==========================================================
    # Slider Builders
    # ==========================================================
    def _build_angle_slider(self, label, minv, maxv, row, callback):
        ttk.Label(self.root, text=f"{label} (deg)").grid(
            row=row, column=0, sticky="w"
        )

        var = tk.DoubleVar(value=0.0)

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
        ttk.Label(self.root, text=label).grid(
            row=row, column=0, sticky="w"
        )

        var = tk.DoubleVar(value=minv)

        slider = ttk.Scale(
            self.root,
            from_=minv,
            to=maxv,
            variable=var,
            command=lambda _: callback(var.get())
        )
        slider.grid(row=row, column=1, sticky="ew", padx=10)

        value_label = ttk.Label(self.root, text=f"{minv:.4f}")
        value_label.grid(row=row, column=2, sticky="w")

        if label == "KP Roll":
            self.kp_roll_value_label = value_label
        elif label == "KP Pitch":
            self.kp_pitch_value_label = value_label
        elif label == "KD Roll":
            self.kd_roll_value_label = value_label
        elif label == "KD Pitch":
            self.kd_pitch_value_label = value_label
        elif label == "KI Roll":
            self.ki_roll_value_label = value_label
        elif label == "KI Pitch":
            self.ki_pitch_value_label = value_label

    # ==========================================================
    # Update Callbacks
    # ==========================================================
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

    def _update_kd_roll(self, v):
        self.kd_roll = v
        self.kd_roll_value_label.config(text=f"{v:.4f}")

    def _update_kd_pitch(self, v):
        self.kd_pitch = v
        self.kd_pitch_value_label.config(text=f"{v:.4f}")

    def _update_ki_roll(self, v):
        self.ki_roll = v
        self.ki_roll_value_label.config(text=f"{v:.4f}")        
    
    def _update_ki_pitch(self, v):
        self.ki_pitch = v
        self.ki_pitch_value_label.config(text=f"{v:.4f}")

    def _toggle_enable(self):
        self.enable = 1.0 if self.enable_var.get() else 0.0

    # ==========================================================
    # Main Loop
    # ==========================================================
    def loop(self):

        msg = Float64MultiArray()

        msg.data = [
            self.desired_roll,   # 0
            self.desired_pitch,  # 1
            self.kp_roll,        # 2
            self.kp_pitch,       # 3
            self.kd_roll,        # 4
            self.kd_pitch,       # 5
            self.ki_roll,        # 6
            self.ki_pitch,       # 7
            self.enable          # 8
        ]

        self.pub.publish(msg)

        rclpy.spin_once(self, timeout_sec=0.0)

        self.root.update_idletasks()
        self.root.update()

        self.root.after(50, self.loop)


# ==========================================================
def main():
    rclpy.init()
    gui = BaseAttitudeGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
