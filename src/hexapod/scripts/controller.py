#!/usr/bin/python3

from hexapod.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class DummyNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.create_subscription(JointState, "base/trajectory" ,self.base_trajectory_callback,10)
        self.create_subscription(JointState, "hip/trajectory" ,self.hip_trajectory_callback,10)
        self.create_subscription(JointState, "knee/trajectory" ,self.knee_trajectory_callback,10)
        self.create_subscription(Float64, "base/position" ,self.base_position_callback,10)
        self.create_subscription(Float64, "hip/position" ,self.hip_position_callback,10)
        self.create_subscription(Float64, "knee/position" ,self.knee_position_callback,10)
        self.base_cmd_pos_pub = self.create_publisher(Float64, "base/cmd_pos" ,10)
        self.hip_cmd_pos_pub = self.create_publisher(Float64, "hip/cmd_pos" ,10)
        self.knee_cmd_pos_pub = self.create_publisher(Float64, "knee/cmd_pos" ,10)

    def base_trajectory_callback(self,msg):
        pass
    def hip_trajectory_callback(self,msg):
        pass
    def knee_trajectory_callback(self,msg):
        pass
    def base_position_callback(self,msg):
        pass
    def hip_position_callback(self,msg):
        pass
    def knee_position_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
