#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class DistanceReader:
    def _init_(self):
        
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose,self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self,msg):
        rospy.loginfo("Turtle Position: %s %s", msg.x, msg.y)

        ########## YOUR CODE GOES HERE ##########
        # Calculate the distance the turtle has travelled and publish it
        pos_x=msg.x
        pos_y=msg.y 

        distance=math.sqrt((pos_x)*2+(pos_y)*2)
        rospy.loginfo("Total distance: %s", distance)
        self.distance_publisher.publish(distance)
        ###########################################

if _name_ == '_main_': 

    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass