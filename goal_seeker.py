import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.current_position = Point()
        self.goal_position = Point()
        self.goal_active = False

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
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
        self.current_position.x = msg.x
        self.current_position.y = msg.y
        self.current_position.theta = msg.theta

    def goal_position_callback(self, msg):
        self.goal_position = msg
        self.goal_active = True

    def timer_callback(self, event):
        if self.goal_active:
            distance_to_goal = math.sqrt((self.goal_position.x - self.current_position.x)**2 + 
                                         (self.goal_position.y - self.current_position.y)**2)
            angle_to_goal = math.atan2(self.goal_position.y - self.current_position.y, 
                                       self.goal_position.x - self.current_position.x)
            angle_diff = angle_to_goal - self.current_position.theta

            # Normalize angle_diff to be within [-pi, pi]
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            if angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            twist_msg = Twist()

            if abs(angle_diff) > 0.01:
                # Rotate to face the goal
                twist_msg.angular.z = 1.0 if angle_diff > 0 else -1.0
            elif distance_to_goal > 0.1:
                # Move straight towards the goal
                twist_msg.linear.x = 1.0
            else:
                # Goal reached
                self.goal_active = False
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0

            self.velocity_publisher.publish(twist_msg)

if __name__ == '__main__': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
