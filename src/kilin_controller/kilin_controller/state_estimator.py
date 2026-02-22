import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')

        # --- State ---
        self.base_quat = np.array([1,0,0,0])
        self.base_rpy = np.zeros(3)  # roll, pitch, yaw
        self.contact_state = {'FL':False, 'FR':False, 'RL':False, 'RR':False}

        self.hip_joints = ['FL_hip','FR_hip','RL_hip','RR_hip']
        self.susp_joints = ['FL_suspension','FR_suspension','RL_suspension','RR_suspension']
        self.suspensions = {j:0.0 for j in self.susp_joints}
        self.hip_efforts = {j:0.0 for j in self.hip_joints}

        self.susp_on = 0.002
        self.susp_off = 0.001
        self.eff_on = 30.0
        self.eff_off = 25.0

        # --- Mass info ---
        self.base_mass = 18.4
        self.leg_masses = {'FL':3.29, 'FR':3.29, 'RL':3.29, 'RR':3.29}  # 1.4+0.14+0.25+1.6
        self.leg_positions = {
            'FL': np.array([0.15, 0.10, 0.0]),
            'FR': np.array([0.15, -0.10, 0.0]),
            'RL': np.array([-0.15, 0.10, 0.0]),
            'RR': np.array([-0.15, -0.10, 0.0])
        }

        # --- Center Of Mass ---
        self.com = np.array([0.0,0.0,0.0])

        # Subscribers
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Publishers
        self.contact_pub = self.create_publisher(Float64MultiArray, '/contact_state', 10)
        self.rpy_pub = self.create_publisher(Float64MultiArray, '/base_rpy', 10)
        self.com_pub = self.create_publisher(Float64MultiArray, '/com_projection', 10)

        self.get_logger().info("StateEstimator started")

    def imu_cb(self, msg: Imu):
        qx,qy,qz,qw = msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w
        self.base_quat = np.array([qw,qx,qy,qz])
        self.base_rpy = self.quat_to_rpy(self.base_quat)
        self.publish_rpy()

    def joint_cb(self, msg: JointState):
        for name,pos,eff in zip(msg.name,msg.position,msg.effort):
            if name in self.susp_joints:
                self.suspensions[name] = pos
            if name in self.hip_joints:
                self.hip_efforts[name] = eff
        self.update_contact()
        self.publish_contact()
        self.compute_com()

    def update_contact(self):
        for leg in ['FL','FR','RL','RR']:
            susp = self.suspensions[f'{leg}_suspension']
            eff  = self.hip_efforts[f'{leg}_hip']
            prev = self.contact_state[leg]
            if not prev:
                self.contact_state[leg] = (abs(susp)>self.susp_on or abs(eff)>self.eff_on)
            else:
                self.contact_state[leg] = not (abs(susp)<self.susp_off and abs(eff)<self.eff_off)

    def publish_contact(self):
        msg = Float64MultiArray()
        msg.data = [1.0 if self.contact_state[leg] else 0.0 for leg in ['FL','FR','RL','RR']]
        self.contact_pub.publish(msg)

    def publish_rpy(self):
        msg = Float64MultiArray()
        msg.data = list(self.base_rpy)
        self.rpy_pub.publish(msg)
    
    # ------------------------------------------------------------------
    # COM calculation
    # ------------------------------------------------------------------
    def compute_com(self):
        # Base COM in body frame
        total_mass = self.base_mass + sum(self.leg_masses.values())
        com = self.base_mass * np.array([0.0,0.0,0.0])
        for leg, mass in self.leg_masses.items():
            com += mass * self.leg_positions[leg]
        com /= total_mass
        self.com = com

        # Project COM onto stance plane if ≥3 legs on ground
        stance_legs = [leg for leg,state in self.contact_state.items() if state]
        if len(stance_legs) >= 3:
            p1 = self.leg_positions[stance_legs[0]]
            p2 = self.leg_positions[stance_legs[1]]
            p3 = self.leg_positions[stance_legs[2]]
            n = np.cross(p2-p1, p3-p1)
            norm_n = np.linalg.norm(n)
            if norm_n > 1e-6:
                n = n / norm_n
                distance = np.dot(n, self.com - p1)
                com_proj = self.com - distance*n
                self.publish_com(com_proj)
        else:
            self.publish_com(self.com)

    def publish_com(self, com):
        msg = Float64MultiArray()
        msg.data = com.tolist()
        self.com_pub.publish(msg)


    @staticmethod
    def quat_to_rpy(q):
        w,x,y,z = q
        roll  = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(np.clip(2*(w*y-z*x),-1.0,1.0))
        yaw   = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return np.array([roll,pitch,yaw])

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
