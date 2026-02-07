import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk

class CustomJointGUI(Node):
    def __init__(self):
        super().__init__('custom_joint_gui')

        # Publisher
        self.pub = self.create_publisher(Float64MultiArray, '/custom_joint_cmd', 10)

        # GUI
        self.root = tk.Tk()
        self.root.title("Custom Joint Command GUI")

        self.joint_vars = {}
        joints = ['FL', 'FR', 'RL', 'RR']
        for i, leg in enumerate(joints):
            tk.Label(self.root, text=f"{leg} hip (deg)").grid(row=i, column=0)
            var = tk.DoubleVar(value=0.0)
            self.joint_vars[leg] = var
            slider = ttk.Scale(self.root, from_=-180, to=180, variable=var, orient='horizontal',
                               command=lambda val, leg=leg: self.publish_cmd())
            slider.grid(row=i, column=1, padx=10, pady=5)

        # Enable checkbox
        self.enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.root, text="Enable Controller",
                        variable=self.enable_var, command=self.publish_cmd).grid(row=4, column=0, columnspan=2)

        # Timer to spin ROS
        self.timer = self.create_timer(0.05, self._spin)

    def _spin(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.update_idletasks()
        self.root.update()

    def publish_cmd(self):
        if not self.enable_var.get():
            return
        msg = Float64MultiArray()
        msg.data = [self.joint_vars[leg].get() * 3.14159 / 180.0 for leg in ['FL','FR','RL','RR']]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CustomJointGUI()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
