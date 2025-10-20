#!/usr/bin/python3

from hexapod.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class DummyNode(Node):
    def __init__(self):

        super().__init__('forward_kinematic_node')
        self.create_subscription(Float64, "base/position" ,self.base_pose_callback,10)
        self.create_subscription(Float64, "hip/position" ,self.hip_pose_callback,10)
        self.create_subscription(Float64, "knee/position" ,self.knee_pose_callback,10)
        self.foot_pose_pub = self.create_publisher(PoseStamped, 'foot/pose',10)

    def base_pose_callback(self,msg):
        pass

    def hip_pose_callback(self,msg):
        pass

    def knee_pose_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
