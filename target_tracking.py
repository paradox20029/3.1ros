#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        # Initialize publisher and subscriber with your robot's name
        self.cmd_vel_pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1,DetectionArray, self.tag_callback, queue_size=1)
        rospy.spin() # Spin forever but listen to message callbacks

    # April Tag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keeps moving
with the last velocity command
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0
            cmd_msg.omega = 0.2
            self.cmd_vel_pub.publish(cmd_msg)
            return
        self.stop_robot()
        x = detections[0].transform.translation.x
        y = detections[0].transform.translation.y
        z = detections[0].transform.translation.z
        rospy.loginfo("x, y, z: %f, %f, %f", x, y, z)
        rospy.sleep(1)
        if x > 0.05:
          cmd_msg = Twist2DStamped()
          cmd_msg.header.stamp = rospy.Time.now()
          cmd_msg.v = 0
          cmd_msg.omega = -0.4
          self.cmd_vel_pub.publish(cmd_msg)
          rospy.sleep(0.4)
          self.stop_robot()
        if x < -0.05:
          cmd_msg = Twist2DStamped()
          cmd_msg.header.stamp = rospy.Time.now()
          cmd_msg.v = 0
          cmd_msg.omega = 0.4
          self.cmd_vel_pub.publish(cmd_msg)
          rospy.sleep(0.4)
          self.stop_robot()
if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass