#!/usr/bin/env python

import rospy
import tf
import turtlesim.srv
import time
import math
import sys
import csv
import actionlib

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA
from move_base_msgs.msg import *

base_position = 'map'
displaced_position = 'base_link'
fieldnames=['name', 'posX', 'posY', 'posZ', 'quat0', 'quat1', 'quat2', 'quat3', 'markerNum', 'type', 'radius']

CIRCLE = 'circle'
POSE = 'pose'

class Annotator:
        def draw_marker(self, pose, name):
                print "Drawing marker"
                print(pose)
                self._num_markers += 1
                marker = Marker(type=Marker.SPHERE, id=self._num_markers,
			    lifetime=rospy.Duration(),
			    pose=pose,
			    scale=Vector3(0.06, 0.06, 0.06),
			    header=Header(frame_id=base_position),
			    color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=name)
		self._marker_publisher.publish(marker)
                self._num_markers += 1
                marker_text = Marker(type=Marker.TEXT_VIEW_FACING, id=self._num_markers,
                        lifetime = rospy.Duration(), pose=pose, scale=Vector3(0.6, 0.6, 0.6),
                        header=Header(frame_id=base_position),
                        color=ColorRGBA(1.0, 0.0, 0.0, 0.8), text=name)
                self._marker_publisher.publish(marker)

        def draw_region(self, pose, name, radius):
                print "Drawing region"
                print(pose)
                self._num_markers += 1
                marker = Marker(type=Marker.CYLINDER, id=self._num_markers,
                        lifetime=rospy.Duration(), pose=pose,
                        scale=Vector3(radius, radius, 0.06),
                        header=Header(frame_id=base_position),
                        color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=name)
                self._marker_publisher.publish(marker)
                self._num_markers += 1
                marker_text = Marker(type=Marker.TEXT_VIEW_FACING, id=self._num_markers,
                        lifetime = rospy.Duration(), pose=pose, scale=Vector3(0.6, 0.6, 0.6),
                        header=Header(frame_id=base_position),
                        color=ColorRGBA(1.0, 0.0, 0.0, 0.8), text=name)
                self._marker_publisher.publish(marker)

	def place_marker(self, name, regionType, regionSize):
                now = rospy.Time.now()
                # This is made up of a position and a quarternion
                self._listener.waitForTransform(base_position, displaced_position, now, rospy.Duration(20.0))
                (pos, quat) = self._listener.lookupTransform(base_position, displaced_position, rospy.Time(0))

		pointPosition = Point(pos[0], pos[1], pos[2])
		quaternion = Quaternion(quat[0], quat[1], quat[2], quat[3])
		self._num_markers += 1
		print 'putting down a marker'
                if regionType == CIRCLE:
                    self.draw_region(Pose(pointPosition, quaternion), name, regionSize)
                else:
                    self.draw_marker(Pose(pointPosition, quaternion), name)

                newMarker = {}
                newMarker['name'] = name
                newMarker['posX'] = pos[0]
                newMarker['posY'] = pos[1]
                newMarker['posZ'] = pos[2]
                newMarker['quat0'] = quat[0]
                newMarker['quat1'] = quat[1]
                newMarker['quat2'] = quat[2]
                newMarker['quat3'] = quat[3]
                newMarker['markerNum'] = self._num_markers
                newMarker['type'] = regionType
                newMarker['radius'] = regionSize
                self._markers[name] = newMarker
                self.write_data(newMarker)

	def write_data(self, marker):
		with open(self._filename, 'a') as csvfile:
			writer = csv.DictWriter(csvfile,
                                fieldnames=fieldnames,
                                delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			writer.writerow(marker)

	def go_to_marker(self, name):
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
                finished_on_time = self._action_client.wait_for_result(rospy.Duration.from_sec(30))
                if not finished_on_time:
                    self._action_client.cancel_goal()
                    print "Timed out acheiving goal"
                else:
                    if self._action_client.get_state() == GoalStatus.SUCCEEDED:
                        print "Success!"
                    else:
                        print "Failure"

	def __init__(self, filename):
		self._markers = {}
		self._filename = filename
		self._num_markers = 0
                self._goal_id=0

                # Set up node for annotater
                rospy.init_node('turtlebot_annotater')
		# Create a TF listener
		self._listener = tf.TransformListener()
		self._rate = rospy.Rate(10.0)
		self._marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=2)

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
                        if my_row['type'] == CIRCLE:
                            self.draw_region(pose, my_row['name'], my_row['radius'])
                        else:
                            self.draw_marker(pose, my_row['name'])
                print "file read"
                print self._markers

                # Set up action client to send goals to
                self._action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                # Note, if acml_demo.launch is not running, this will take forever
                self._action_client.wait_for_server()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Pass file location as argument to command line."
        print "Defaulting to markers.cv"
        filename = 'markers.cv'
    else:
	filename = sys.argv[1]
    annotate = Annotator(filename)
    while(1):
            terms = raw_input('Save or go to marker? (s name) save pose, ' +
                    '(r name) save region, (g name) go to marker, (l) list: ')
            args = terms.split(" ")
            if len(args) < 1 or (
                    len(args) < 2 and (args[0] == 's' or args[0] == 'g' or args[0] == 'r')):
                    print "I need a name to do that"
            elif args[0] == 's':
                    annotate.place_marker(args[1], POSE, 0)
            elif args[0] == 'r':
                    radius = float(raw_input('Enter radius of region as a float: '))
                    annotate.place_marker(args[1], CIRCLE, radius)
            elif args[0] == 'g':
                    annotate.go_to_marker(args[1])
            elif args[0] == 'l':
                    for marker in self._markers:
                        print marker['name'], "\t of type ", marker['type']
