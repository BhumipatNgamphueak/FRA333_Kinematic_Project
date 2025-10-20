#!/usr/bin/python3

from hexapod.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class DummyNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')
        self.create_subscription(JointState, "foot/trajectory" ,self.foot_trajectory_callback,10)
        self.base_trajectory_pub = self.create_publisher(JointState, "base/trajectory" ,10)
        self.hip_trajectory_pub = self.create_publisher(JointState, "hip/trajectory" ,10)
        self.knee_trajectory_pub = self.create_publisher(JointState, "knee/trajectory" ,10)


    def foot_trajectory_callback(self,msg):
        pass
 


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
