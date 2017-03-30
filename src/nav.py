import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

class Navigation():
    def __init__(self):
        #goal1 is the corner of the room with the workstations
        #The robot will then vist the other corners of the room clockwise
        self.goal1_x = 2.73
        self.goal1_y = 2.10
        self.goal2_x = 2.47
        self.goal2_y = -1.47
        self.goal3_x = -6.15
        self.goal3_y = -0.37
        self.goal4_x = -5.55
        self.goal4_y = 4.14

        rospy.init_node('Navigation')

        self.cornerReached = goToCorner(self.goal1_x, self.goal1_y)
        if self.cornerReached:
            print("Corner 1 reached.")
        else:
            print("Failed to reach Corner 1.")

        self.cornerReached = goToCorner(self.goal2_x, self.goal2_y)
        if self.cornerReached:
            print("Corner 2 reached.")
        else:
            print("Failed to reach Corner 2.")

        self.cornerReached = goToCorner(self.goal3_x, self.goal3_y)
        if self.cornerReached:
            print("Corner 3 reached.")
        else:
            print("Failed to reach Corner 3.")

        self.cornerReached = goToCorner(self.goal4_x, self.goal4_y)
        if self.cornerReached:
            print("Corner 4 reached.")
        else:
            print("Failed to reach corner 4.")

    def goToCorner(self, x, y):
        action = actionlib.SimpleActionClient("moveBot", MoveBaseAction)

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            print("Waiting for move_base action server to respond.")

        corner = MoveBaseGoal()

        corner.target_pose.header.frame_id = "map"
        corner.target_pose.header.stamp = rospy.Time.now()

        corner.target_pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        action.send_goal(corner)
        print("Sent goal " + corner)

        action.wait_for_result(rospy.Duration(60))

        if action.get_state() == GoalStatus.SUCCEEDED:
            print("Corner reached")
            return True
        else:
            print("Failed to reach corner.")
            return False

#if __name__ == 'main':
#    try:
#        Navigation()
#        print("Started nav")
#        rospy.spin()
#    except rospy.ROSInterruptException:
#        print("ROSInterruptException. Navigation failed.")

Nav = Navigation()
