import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DummySimulator(Node):
    def __init__(self):
        super().__init__('dummy_simulator')

        # 訂閱 joint_command
        self.sub = self.create_subscription(
            JointState,
            '/joint_command',   # 接收 commander 發的指令
            self.command_callback,
            10
        )

        # 發佈 joint_state
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def command_callback(self, msg: JointState):
        # 原封不動發給 /joint_states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = msg.name
        joint_state.position = msg.position
        joint_state.velocity = msg.velocity
        joint_state.effort = msg.effort

        self.pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = DummySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
