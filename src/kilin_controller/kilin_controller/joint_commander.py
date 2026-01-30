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
            'FL_Joint0', 'FL_Joint1', 'FL_Joint2', 'FL_Joint3',
            'FR_Joint0', 'FR_Joint1', 'FR_Joint2', 'FR_Joint3',
            'RL_Joint0', 'RL_Joint1', 'RL_Joint2', 'RL_Joint3',
            'RR_Joint0', 'RR_Joint1', 'RR_Joint2', 'RR_Joint3',
        ])
        self.joint_names = self.get_parameter('joint_names').value
        
        # Only move these joints
        self.active_joints = ['FL_Joint0', 'FR_Joint0', 'RL_Joint0', 'RR_Joint0']
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