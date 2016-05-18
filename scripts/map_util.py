#!/usr/bin/env python

import rospy
import tf
import turtlesim.srv
import time
import math
import sys
import csv
import actionlib

from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA
from move_base_msgs.msg import *

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
        specificGoal = MoveBaseGoal()
        specificGoal.target_pose.header.frame_id = 'map'
        specificGoal.target_pose.header.stamp = rospy.Time.now()

        pose = Pose(point_position, quaternion)
        specificGoal.target_pose.pose = pose

        self._action_client.send_goal(specificGoal)
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
        self._filename = filename
        self._num_markers = 0
        self._goal_id=0

        # Set up node for annotater
        rospy.init_node('turtlebot_map')
        # Create a TF listener
        self._listener = tf.TransformListener()
        self._rate = rospy.Rate(10.0)

        # Wait until subscribers notice new publisher
        rospy.sleep(1)

        print "opening file"
        # Load in any previously saved markers
        with open(self._filename, 'ab+') as csvfile:
            reader = csv.DictReader(csvfile,
                    fieldnames=fieldnames, delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                my_row = dict()
                my_row['name'] = row['name']
                my_row['posX'] = float(row['posX'])
                my_row['posY'] = float(row['posY'])
                my_row['posZ'] = float(row['posZ'])
                my_row['quat0'] = float(row['quat0'])
                my_row['quat1'] = float(row['quat1'])
                my_row['quat2'] = float(row['quat2'])
                my_row['quat3'] = float(row['quat3'])
                my_row['radius'] = float(row['radius'])
                my_row['type'] = row['type']
                self._markers[row['name']]=my_row
                pose = Pose(Point(my_row['posX'], my_row['posY'], my_row['posZ']),
                        Quaternion(my_row['quat0'], my_row['quat1'], my_row['quat2'], my_row['quat3']))
        print "file read"
        print self._markers

        # Set up action client to send goals to
        self._action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Note, if acml_demo.launch is not running, this will take forever
        self._action_client.wait_for_server()
