#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool
#if I want to write a publisher I need to import the type of message that is used by the topic
#if not already done remember to add the dependancy in the package.xml file to geometry_msgs
import time

class controller(Node):
    def __init__(self):
        super().__init__("Controller2") #this defines the node name
        self.get_logger().info("Controller node has been started") 
        #this generates a print as soon as the node is started
        self.res_sub_ = self.create_subscription(Bool,'/reset',self.reset_callback,10)
        self.res_sub_  # prevent unused variable warning
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_topic", 10) 
        #the entries are the message type, the name of the topic and the queue size,
        #that is a buffer that indicates the number of messages
        self.timer_ = self.create_timer(1.0, self.send_velocity_command) 
        #In order to send the message we create a timer that every 1 seconds calls the function 
        self.msg = Twist()
        self.num_of_cycles = 1
        self.time_to_move = 1.0
        self.flag = False

    def reset_callback(self, msg : Bool):
        if msg.data:
            self.flag = True

    def send_velocity_command(self): #now we want to call this function every 1 seconds
        #the movement iterates 4 time, one for every direction           
        for iteration in range(1,5):
            #each direction iterates for the amount of time required
            start_time = time.time()
            while time.time()-start_time < self.time_to_move:
                if iteration == 1:
                    self.msg.linear.x = 1.0 #Now I can assign some values to the entries of the twist message called msg
                    self.msg.linear.y = 0.0
                    self.cmd_vel_pub_.publish(self.msg) #now we use the publish method of the publisher created before to publish the message
                elif iteration == 2:
                    self.msg.linear.x = 0.0
                    self.msg.linear.y = 1.0
                    self.cmd_vel_pub_.publish(self.msg)
                elif iteration == 3:
                    self.msg.linear.x = -1.0
                    self.msg.linear.y = 0.0
                    self.cmd_vel_pub_.publish(self.msg)
                elif iteration == 4:
                    self.msg.linear.x = 0.0
                    self.msg.linear.y = -1.0
                    self.cmd_vel_pub_.publish(self.msg)
                time.sleep(1.0)

        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.cmd_vel_pub_.publish(self.msg) #with this I stop the robot after the movement
        self.time_to_move += 1.0 #I increase the variable that sets the time of movement
        self.num_of_cycles += 1

def main(args=None):
    rclpy.init(args=args)
    node = controller() #we create the node
    rclpy.spin(node) #this is necessary to keep the node alive and use the timer, otherwise the main function will end immediatly
    node.destroy_node()
    rclpy.shutdown()
    #once you have finisher writing the node remember to install it in setup.py in order to make it possible to run the node using the ros2 run command

if __name__ == '__main__':
    main()