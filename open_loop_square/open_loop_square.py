#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range
 
class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        self.tick_count = 0
        self.front_dis = 0
        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/duckie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/duckie/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.move_robot()

    def encoder_callback(self, msg):
        self.tick_count = msg.data

    def range_callback(self, msg):
        self.front_dis = msg.range

   
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()

    # Robot drives in a square and then stops
    def move_robot(self):
        for i in range(4):
         self.move_straight()
        self.rotate_in_place()
        self.stop_robot()

    def move_straight(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 1 # striaght line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(1) # straight line driving time

    def rotate_in_place(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0 # striaght line velocity
        self.cmd_msg.omega = 1.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turn!")
        rospy.sleep(1) # straight line driving time
        self.stop_robot()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass