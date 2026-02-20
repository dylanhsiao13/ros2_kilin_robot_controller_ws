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
        self.kp_roll = 0.005
        self.kp_pitch = 0.005
        self.kd_roll = 0.0
        self.kd_pitch = 0.0
        self.ki_roll = 0.0
        self.ki_pitch = 0.0
        self.alpha_dz = 0.15
        self.enable = True

        # ==================================================
        # State input
        # ==================================================
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_e_roll = 0.0
        self.prev_e_pitch = 0.0
        self.int_roll = 0.0
        self.int_pitch = 0.0
        self.contact = {
            'FL': True,
            'FR': True,
            'RL': True,
            'RR': True
        }
        self.dz_filtered = {
            "FL": 0.0,
            "FR": 0.0,
            "RL": 0.0,
            "RR": 0.0
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
        self.timer = self.create_timer(0.1, self.control_loop) #10 Hz
        self.get_logger().info("BaseAttitudeController (topic-based) started")

    # --------------------------------------------------
    def cmd_cb(self, msg: Float64MultiArray):
        """
        data = [
          desired_roll,
          desired_pitch,
          kp_roll,
          kp_pitch,
          kd_roll,
          kd_pitch,
          ki_roll,
          ki_pitch,
          enable (0/1)
        ]
        """
        if len(msg.data) < 8:
            self.get_logger().warn("Received incomplete base_attitude_cmd message")
            return

        self.desired_roll = msg.data[0]
        self.desired_pitch = msg.data[1]
        self.kp_roll = msg.data[2]
        self.kp_pitch = msg.data[3]
        self.kd_roll = msg.data[4]
        self.kd_pitch = msg.data[5]
        self.ki_roll = msg.data[6]
        self.ki_pitch = msg.data[7]
        self.enable = msg.data[8] > 0.5

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

        # -------- attitude error(PID) --------
        e_roll =  self.desired_roll - self.roll
        e_pitch =  self.desired_pitch - self.pitch

        dt=1/10.0
        de_roll = (e_roll - self.prev_e_roll) / dt
        de_pitch = (e_pitch - self.prev_e_pitch) / dt
        self.prev_e_roll = e_roll
        self.prev_e_pitch = e_pitch

        self.int_roll  += e_roll  * dt
        self.int_pitch += e_pitch * dt

        msg = Float64MultiArray()
        msg.data = [self.desired_roll , self.roll, e_roll, self.desired_pitch , self.pitch, e_pitch]
        self.error_pub.publish(msg)

        # -------- dz command --------
        raw_dz = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Pitch correction
        raw_dz['FL'] += -self.kp_pitch * e_pitch-self.kd_pitch * de_pitch-self.ki_pitch * self.int_pitch
        raw_dz['FR'] += -self.kp_pitch * e_pitch-self.kd_pitch * de_pitch-self.ki_pitch * self.int_pitch
        raw_dz['RL'] += self.kp_pitch * e_pitch+self.kd_pitch * de_pitch+self.ki_pitch * self.int_pitch
        raw_dz['RR'] += self.kp_pitch * e_pitch+self.kd_pitch * de_pitch+self.ki_pitch * self.int_pitch

        # Roll correction
        raw_dz['FL'] += self.kp_roll * e_roll+self.kd_roll * de_roll+self.ki_roll * self.int_roll
        raw_dz['FR'] += -self.kp_roll * e_roll-self.kd_roll * de_roll-self.ki_roll * self.int_roll
        raw_dz['RL'] +=  self.kp_roll * e_roll+self.kd_roll * de_roll+self.ki_roll * self.int_roll
        raw_dz['RR'] +=  -self.kp_roll * e_roll-self.kd_roll * de_roll-self.ki_roll * self.int_roll

        # Low-pass filter
        for leg in raw_dz:
            self.dz_filtered[leg] = (
                self.alpha_dz * raw_dz[leg] +
                (1 - self.alpha_dz) * self.dz_filtered[leg]
            )

        # Apply only to contact legs
        dz_cmd = Float64MultiArray()
        dz_cmd.data = [
            self.dz_filtered['FL'] if self.contact['FL'] else 0.0,
            self.dz_filtered['FR'] if self.contact['FR'] else 0.0,
            self.dz_filtered['RL'] if self.contact['RL'] else 0.0,
            self.dz_filtered['RR'] if self.contact['RR'] else 0.0,
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
