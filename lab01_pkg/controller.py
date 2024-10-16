import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Controller(Node):
    
    def __init__(self):
        super().__init__("Controller2")
        self.get_logger().info("Controller node has been started")
        # Subscriber to /reset topic
        self.res_sub_ = self.create_subscription(Bool, '/reset', self.reset_callback, 10)
        self.res_sub_ 
        # Publisher to /cmd_topic
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_topic", 10)
        #I initialize the period as a parameter
        self.declare_parameter("period", 0.1)   
        self.period = self.get_parameter("period").get_parameter_value().double_value
        # Timer to control the movement
        self.timer_ = self.create_timer(self.period, self.send_velocity_command)
        # Message initialization
        self.msg = Twist()
        # Movement control variables
        self.time_to_move = 1 #determine how long the movement last
        self.counter = 0 #since we are publishing 1 every second we can use a counter to know how long the robot has moved 
        self.flag = False  # Used for resetting the movement
        self.direction_index = 0  # Index to track which direction the robot is moving in
        self.directions = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]  # [x, y] directions

    def reset_callback(self, msg: Bool):
        if msg.data:
            self.flag = True

    def send_velocity_command(self):
        # Reset logic if flag is set
        if self.flag:
            self.time_to_move = 1
            self.flag = False #reset the flag
            self.get_logger().info("Movement reset")
            self.direction_index = 0  # Reset direction to the beginning
            self.counter = 0 #Reset the counter for the restart of the cycle
        # Check if the robot has completed movement in the current direction
        if self.counter < self.time_to_move:
            # Set the velocity for the current direction (velocity is 1 m/s in either x or y direction)
            self.msg.linear.x = self.directions[self.direction_index][0]
            self.msg.linear.y = self.directions[self.direction_index][1]
            # Publish the velocity command
            self.cmd_vel_pub_.publish(self.msg)
            self.counter += self.period #increase the counter
        else:
            # Move to the next direction
            self.direction_index = self.direction_index + 1  # Cycle through 4 directions
            # If the cycle is complete (i.e., after 4 directions)
            if self.direction_index == 4:
                self.time_to_move += 1  # Increment time for the next cycle
                self.direction_index = 0
                self.get_logger().info(f"Completed cycle")
            # Reset the counter for the next direction
            self.counter = 0
            
def main(args=None):

    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
