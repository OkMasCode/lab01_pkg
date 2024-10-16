import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class localization(Node):

    def __init__(self):
        super().__init__('localization')
        #subscription to the reset topic
        self.sub_res_ = self.create_subscription(Bool,'/reset',self.reset_callback,10)
        self.sub_res_
        #subscription to the cmd_topic topic
        self.sub_cmd_vel_ = self.create_subscription(Twist,'/cmd_topic',self.velocity_callback,10)
        self.sub_cmd_vel_
        #publisher on the pose topic
        self.pos_pub_ = self.create_publisher(Pose, "/pose", 10)
        #initialization of variables
        self.x = 0.0
        self.y = 0.0
        self.declare_parameter("period", 0.1)
        self.period = self.get_parameter("period").get_parameter_value().double_value

    def reset_callback(self, msg : Bool):
        #reset of variables when the reset message is received
        if msg.data:
            self.x = 0.0
            self.y = 0.0
            self.get_logger().info("Position Reset")
        
    def velocity_callback(self, msg : Twist):
        #create a pose message to publish the estimate pose
        #Since the publish happens into the callback of the listener 
        #we do not need a timer, since the callback gets called as soon
        #as it receives a message
        pos = Pose()
        pos.position.x = self.x
        pos.position.y = self.y
        self.pos_pub_.publish(pos)
        self.get_logger().info(f"Pose: ({self.x:.2},{self.y:.2})")
        #we put the update of the position after the publish
        #In this way we publish what is the position before the movement
        update_x = self.period * msg.linear.x
        update_y = self.period * msg.linear.y
        self.x += update_x
        self.y += update_y

def main(args=None):

    rclpy.init(args=args)
    local = localization()
    rclpy.spin(local)
    local.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()