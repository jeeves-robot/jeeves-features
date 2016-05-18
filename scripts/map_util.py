#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from move_base_msgs.msg import *

from load_locations import load_locations

base_position = 'map'
displaced_position = 'base_link'
fieldnames=['name', 'posX', 'posY', 'posZ', 'quat0', 'quat1', 'quat2', 'quat3', 'markerNum', 'type', 'radius']

CIRCLE = 'circle'
POSE = 'pose'

class MapUtil:
    def go_to_marker(self, name, timeout):
        self._goal_id += 1
        marker = self._markers[name]
        point_position = Point(marker['posX'], marker['posY'], marker['posZ'])
        quaternion = Quaternion(marker['quat0'], marker['quat1'], marker['quat2'], marker['quat3'])
        print(point_position)
        print(quaternion)

        # Set up goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        pose = Pose(point_position, quaternion)
        goal.target_pose.pose = pose

        # Send goal
        self._action_client.send_goal(goal)
        finished_on_time = self._action_client.wait_for_result(rospy.Duration.from_sec(timeout))
        if not finished_on_time:
            self._action_client.cancel_goal()
            print "Timed out acheiving goal"
        else:
            if self._action_client.get_state() == GoalStatus.SUCCEEDED:
                print "Success!"
                return True
            else:
                print "Failure"
                return False

    def contains(self, name):
        return name in self._markers

    def __init__(self, filename):
        self._markers = {}
        self._goal_id=0

        # Set up node for map
        rospy.init_node('turtlebot_map')

        # Wait until subscribers notice new publisher
        rospy.sleep(1)

        # Read locations from file
        load_locations(filename, markers)

        print self._markers

        # Set up action client to send goals to
        self._action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Note, if acml_demo.launch is not running, this will take forever
        self._action_client.wait_for_server()
