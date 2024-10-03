#!/usr/bin/env python

import rospy
import unittest
import actionlib
import math
from tortoisebot_waypoints.msg import WaypointAction, WaypointGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatus

class TestWaypoints(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoints')
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointAction)
        self.client.wait_for_server()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_position = None
        self.current_yaw = None
        self.initial_position = None

        # Get parameters from command line
        self.goal_x = float(rospy.get_param('~goal_x', 0.0))
        self.goal_y = float(rospy.get_param('~goal_y', 0.0))
        self.tolerance = float(rospy.get_param('~tolerance', 0.1))
        self.max_test_duration = rospy.Duration(10)  # 10 seconds max for the test

        # Wait for the first odometry message
        self.wait_for_odom()
        self.initial_position = self.current_position

    def wait_for_odom(self):
        timeout = rospy.Time.now() + rospy.Duration(5)  # 5 seconds timeout for odom
        while self.current_position is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertIsNotNone(self.current_position, "Failed to receive odometry data")

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def test_waypoint(self):
        goal = WaypointGoal()
        goal.position.x = self.goal_x
        goal.position.y = self.goal_y
        goal.position.z = 0.0

        self.client.send_goal(goal)
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  
        
        while (rospy.Time.now() - start_time) < self.max_test_duration:
            if self.goal_x == 0.5 and self.goal_y == 0.5:
                # Success case
                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    self.assertTrue(self.check_position(), "Robot did not reach the goal position")
                    self.assertTrue(self.check_orientation(), "Robot's orientation is not correct")
                    rospy.loginfo("Test passed: Robot reached the goal (0.5, 0.5)")
                    return
            else:
                # Failure case
                if self.has_moved_significantly():
                    self.fail(f"Test failed: Robot moved significantly towards unreachable goal ({self.goal_x}, {self.goal_y})")
                    return
            rate.sleep()

        if self.goal_x == 0.5 and self.goal_y == 0.5:
            self.fail("Test failed: Robot did not reach the goal (0.5, 0.5) in time")
        else:
            rospy.loginfo(f"Test passed: Robot did not move significantly towards unreachable goal ({self.goal_x}, {self.goal_y})")

    def check_position(self):
        return (abs(self.current_position.x - self.goal_x) <= self.tolerance and
                abs(self.current_position.y - self.goal_y) <= self.tolerance)

    def check_orientation(self):
        expected_yaw = math.atan2(self.goal_y, self.goal_x)
        yaw_error = abs(self.current_yaw - expected_yaw)
        yaw_error = min(yaw_error, 2*math.pi - yaw_error)  
        return yaw_error < math.pi/2  

## if the robot moves 10cm to the unreachable goal the test fails immediately
    def has_moved_significantly(self):
        distance_moved = math.sqrt((self.current_position.x - self.initial_position.x)**2 +
                                   (self.current_position.y - self.initial_position.y)**2)
        return distance_moved > 0.1  # Consider 10cm as significant movement

if __name__ == '__main__':
    import rostest
    rostest.rosrun('tortoisebot_waypoints', 'test_waypoints', TestWaypoints)