# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def _init_(self):
        
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        self.last_angle = 0
        self.goal_angle = 0
        self.angle_goal_active = False
        self.clockwise_rotation = True

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self, msg):
        self.last_angle = msg.theta

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        goal_angle = msg.data
        if goal_angle != 0:
            # Set the goal angle
            # Ensure the angle is within the range [-pi, pi]
            if goal_angle > math.pi:
                goal_angle -= 2 * math.pi
            if goal_angle < -math.pi:
                goal_angle += 2 * math.pi
            self.goal_angle = goal_angle
            self.angle_goal_active = True
            self.clockwise_rotation = True if self.goal_angle > 0 else False
        else:
            self.angle_goal_active = False

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.dist_goal_active = True if self.goal_distance != 0 else False
        self.forward_movement = True if self.goal_distance > 0 else False

    def timer_callback(self, msg):
        if self.dist_goal_active:

            # Check if the distance goal is reached
            if abs(self.last_distance - self.goal_distance) >= 0.01:
                dist = Twist()
                dist.linear.x = self.goal_distance
                 # Stop the turtle
                self.velocity_publisher.publish(dist)
                self.dist_goal_active = False
            else:

                 # Publish cmd_vel message to move the turtle
                twist_msg = Twist()
                twist_msg.linear.x = 1 if self.forward_movement else -1
                self.velocity_publisher.publish(twist_msg)

        if self.angle_goal_active:

             # Check if the angle goal is reached
            # if abs(self.last_angle - self.goal_angle) >= 0.01:
            #      # Stop the turtle
            #     angle = Twist()
            #     angle.angular.z = self.goal_angle
            #     self.velocity_publisher.publish(angle)
            #     self.angle_goal_active = False
            # else:
            #      # Publish cmd_vel message to rotate the turtle
            #     twist_msg = Twist()
            #     twist_msg.angular.z = 1 if self.clockwise_rotation else -1
            #     self.velocity_publisher.publish(twist_msg)
            # Compute the difference between current angle and goal angle
            angle_diff = self.goal_angle - self.last_angle

            # Handle the wrap-around condition for angle difference
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            if angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Check if the angle goal is reached
            if abs(angle_diff) >= 0.01:
                # Publish cmd_vel message to rotate the turtle
                twist_msg = Twist()
                
                # If there's a full rotation needed first, do that
                if abs(angle_diff) > math.pi:
                    twist_msg.angular.z = 1 if self.clockwise_rotation else -1
                else:
                    # Otherwise, rotate according to the leftover rotation
                    twist_msg.angular.z = angle_diff
                
                self.velocity_publisher.publish(twist_msg)
            else:
                # Stop the turtle
                self.angle_goal_active = False
if _name_ == '_main_': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass