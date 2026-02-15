import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class BaseAttitudeController(Node):

    def __init__(self):
        super().__init__('base_attitude_controller')

        # ==================================================
        # Command input (from GUI / supervisor)
        # ==================================================
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.kp_roll = 0.05
        self.kp_pitch = 0.05
        self.enable = True

        # ==================================================
        # State input
        # ==================================================
        self.roll = 0.0
        self.pitch = 0.0

        self.contact = {
            'FL': True,
            'FR': True,
            'RL': True,
            'RR': True
        }

        # ==================================================
        # Subscribers
        # ==================================================
        self.create_subscription(
            Float64MultiArray,
            '/base_attitude_cmd',
            self.cmd_cb,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/base_rpy',
            self.rpy_cb,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/contact_state',
            self.contact_cb,
            10
        )

        # ==================================================
        # Publisher
        # ==================================================
        self.dz_pub = self.create_publisher(
            Float64MultiArray,
            '/dz_cmd',
            10
        )

        self.error_pub = self.create_publisher(
            Float64MultiArray,
            '/base_attitude_error',
            10
        )
        # ==================================================
        # Control loop
        # ==================================================
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("BaseAttitudeController (topic-based) started")

    # --------------------------------------------------
    def cmd_cb(self, msg: Float64MultiArray):
        """
        data = [
          desired_roll,
          desired_pitch,
          kp_roll,
          kp_pitch,
          enable (0/1)
        ]
        """
        if len(msg.data) < 5:
            return

        self.desired_roll = msg.data[0]
        self.desired_pitch = msg.data[1]
        self.kp_roll = msg.data[2]
        self.kp_pitch = msg.data[3]
        self.enable = msg.data[4] > 0.5

    def rpy_cb(self, msg: Float64MultiArray):
        """
        data = [roll,pitch,yaw]
        """
        self.roll = msg.data[0]
        self.pitch = msg.data[1]
        

    def contact_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            return

        self.contact['FL'] = msg.data[0] > 0.5
        self.contact['FR'] = msg.data[1] > 0.5
        self.contact['RL'] = msg.data[2] > 0.5
        self.contact['RR'] = msg.data[3] > 0.5

    # --------------------------------------------------
    def control_loop(self):

        if not self.enable:
            return

        # -------- attitude error --------
        e_roll =  self.desired_roll - self.roll
        e_pitch =  self.desired_pitch - self.pitch

        msg = Float64MultiArray()
        msg.data = [self.desired_roll , self.roll, e_roll, self.desired_pitch , self.pitch, e_pitch]
        self.error_pub.publish(msg)

        # -------- dz command --------
        dz = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Pitch correction
        dz['FL'] += -self.kp_pitch * e_pitch
        dz['FR'] += -self.kp_pitch * e_pitch
        dz['RL'] += self.kp_pitch * e_pitch
        dz['RR'] += self.kp_pitch * e_pitch        

        # Roll correction
        dz['FL'] += self.kp_roll * e_roll
        dz['FR'] += -self.kp_roll * e_roll
        dz['RL'] +=  self.kp_roll * e_roll
        dz['RR'] +=  -self.kp_roll * e_roll



        # Apply only to contact legs
        dz_cmd = Float64MultiArray()
        dz_cmd.data = [
            dz['FL'] if self.contact['FL'] else 0.0,
            dz['FR'] if self.contact['FR'] else 0.0,
            dz['RL'] if self.contact['RL'] else 0.0,
            dz['RR'] if self.contact['RR'] else 0.0,
        ]

        self.dz_pub.publish(dz_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BaseAttitudeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
