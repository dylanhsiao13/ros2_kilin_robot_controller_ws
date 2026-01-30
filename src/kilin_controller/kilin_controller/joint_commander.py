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
        
        # Get joint names from parameter
        self.declare_parameter('joint_names',[
        'FL_Joint0', 'FL_Joint1', 'FL_Joint2', 'FL_Joint3',
        'FR_Joint0', 'FR_Joint1', 'FR_Joint2', 'FR_Joint3',
        'RL_Joint0', 'RL_Joint1', 'RL_Joint2', 'RL_Joint3',
        'RR_Joint0', 'RR_Joint1', 'RR_Joint2', 'RR_Joint3',
            ]
)

        self.joint_names = self.get_parameter('joint_names').value
        
        self.get_logger().info(f'Joint Commander initialized with joints: {self.joint_names}')
        
    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Example: Sinusoidal motion for testing
        elapsed = time.time() - self.time_start
        msg.position = [math.sin(elapsed) * 0.5 for _ in self.joint_names]
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