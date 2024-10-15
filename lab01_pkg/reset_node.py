import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import math

class reset(Node):

    def __init__(self):
        super().__init__('reset_node')
        #subscription to the pose topic
        self.pose_sub_ = self.create_subscription(Pose,'/pose',self.pose_callback,10)
        self.pose_sub_
        #publisher on the reset topic
        self.reset_pub_ = self.create_publisher(Bool, "/reset", 10)
        
    def pose_callback(self, msg : Pose):
        #generate the message
        res = Bool()
        res.data = False
        #reading the message on the pose topic
        x = msg.position.x
        y = msg.position.y
        #check the distance
        if math.sqrt(x**2 + y**2) >= 6.0:
            res.data = True
            self.get_logger().info("The position and velocity have been reset")
        else:
            res.data = False
        self.reset_pub_.publish(res)

def main(args=None):

    rclpy.init(args=args)
    res = reset()
    rclpy.spin(res)
    res.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()