#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray, WheelEncoderStamped
from sensor_msgs.msg import Range

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)
        self.cmd_msg = Twist2DStamped()
        self.tick_count = 0
        self.front_dis = 0

        self.robot_state = "LANE_FOLLOWING"

        # When shutdown signal is received, run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "shravel" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/duckie/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/duckie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/duckie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/duckie/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        ################################################################

        rospy.spin()  # Spin forever but listen to message callbacks

    # AprilTag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        self.move_robot(msg.detections)

    # Stop robot before node has shut down
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def encoder_callback(self, msg):
        self.tick_count = msg.data

    def range_callback(self, msg):
        self.front_dis = msg.range

    def goal_distance(self, distance, linear_speed):
        init_tick = self.tick_count
        while abs(self.tick_count - init_tick) < (distance * 100):
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = linear_speed
            self.cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(self.cmd_msg)
            rospy.loginfo("Moving Forward!")
        self.stop_robot()

    def goal_angle(self, angle, angular_speed):
        init_tick = self.tick_count
        while abs(self.tick_count - init_tick) < (angle * 25):
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed
            self.cmd_vel_pub.publish(self.cmd_msg)
            rospy.loginfo("Rotating!")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(self.cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            return
        
        detection = detections[0]
        rospy.loginfo(detection)

        if detection.tag_id == 24:
            rospy.loginfo("STOP")
            self.stop_robot()
            rospy.sleep(3)
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            rospy.sleep(2)
            self.set_state("LANE_FOLLOWING")
            rospy.sleep(4)
        elif detection.tag_id == 11 or detection.tag_id == 10:
            self.handle_special_tag(detection)

    def handle_special_tag(self, detection):
        x1 = detection.transform.translation.x
        y1 = detection.transform.translation.y
        z1 = detection.transform.translation.z
        rospy.loginfo(z1)

        if z1 < 0.3:
            rospy.loginfo("STOP")
            rospy.sleep(1)
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            rospy.sleep(1)
            self.goal_distance(2, 1)
            self.stop_robot()
            rospy.sleep(1)
            if detection.tag_id == 11:
                self.goal_angle(0.5, 4)
            else:
                self.goal_angle(0.5, -4)
            self.stop_robot()
            rospy.sleep(1)
            self.goal_distance(2, 1)
            self.set_state("LANE_FOLLOWING")

        self.handle_obstacle()

    def handle_obstacle(self):
        if self.front_dis < 0.1:
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(5)
            if self.front_dis > 0.1 or self.front_dis == 0:
                rospy.loginfo("Clear")
                self.set_state("LANE_FOLLOWING")
                rospy.sleep(2)
                return

            rospy.loginfo("Vehicle in Front")
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(1)
            self.overtake_maneuver()

    def overtake_maneuver(self):
        self.goal_angle(1, 2)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_distance(1, 1)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_angle(1, -2)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_distance(2, 1)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_angle(1, -2)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_distance(1, 1)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_angle(1, 2)
        self.stop_robot()
        rospy.sleep(1)
        self.goal_distance(1, 1)
        self.stop_robot()
        rospy.sleep(1)
        self.set_state("LANE_FOLLOWING")
        rospy.sleep(4)

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
