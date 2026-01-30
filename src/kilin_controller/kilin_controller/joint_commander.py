import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.time_start = time.time()

        # All joints in the robot
        self.declare_parameter('joint_names', [
        "FL_hip", "FL_steering", "FL_suspension", "FL_wheel",
        "FR_hip", "FR_steering", "FR_suspension", "FR_wheel",
        "RL_hip", "RL_steering", "RL_suspension", "RL_wheel",
        "RR_hip", "RR_steering", "RR_suspension", "RR_wheel",
        ])
    
        self.joint_names = self.get_parameter('joint_names').value
        
        # Only move these joints
        self.active_joints = ['FL_hip', 'FR_hip', 'RL_hip', 'RR_hip']
        self.get_logger().info(f'Joint Commander initialized for joints: {self.active_joints}')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        elapsed = time.time() - self.time_start

        # Make sure every value is a valid float
        positions = []
        for joint in self.joint_names:
            if joint in self.active_joints:
                val = math.sin(elapsed) * 0.5
                if not math.isfinite(val):  # sanity check
                    val = 0.0
            else:
                val = 0.0
            positions.append(val)

        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)    

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()