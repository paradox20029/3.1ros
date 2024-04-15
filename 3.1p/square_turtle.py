#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist 
import time 

def move_turtle_square(): 
    rospy.init_node('turtlesim_square_node', anonymous=True)
    
    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    rospy.loginfo("Turtles are great at drawing squares!")

    ########## YOUR CODE GOES HERE ##########

    Move_forward = Twist()
    Move_forward.linear.x = 1

    Move_left = Twist()
    Move_left.linear.y = 1

    Move_backward = Twist()
    Move_backward.linear.x = -1

    Move_right = Twist()
    Move_right.linear.y = -1

    while True:
        velocity_publisher.publish(Move_forward)
        time.sleep(1)

        # velocity_publisher.publish(Twist())

        velocity_publisher.publish(Move_left)
        time.sleep(1)

        # velocity_publisher.publish(Twist())

        velocity_publisher.publish(Move_backward)
        time.sleep(1)

        # velocity_publisher.publish(Twist())

        velocity_publisher.publish(Move_right)
        time.sleep(1)

        # velocity_publisher.publish(Twist())
    ###########################################

if _name_ == '_main_': 

    try: 
        move_turtle_square() 
    except rospy.ROSInterruptException: 
        pass