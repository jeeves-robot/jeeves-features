import rospy
from geometry_msgs.msg import Twist
import time

### Note: This file provides a variety of different movement
### modes that the robot can take.

# Some frequently used twist commands we will want to publish

NAV_TOPIC = '/cmd_vel_mux/input/navi'

def twist_forward():
    twistForward = Twist()
    twistForward.linear.x = 0.05
    return twistForward

def twist_backward():
    twistBackward = Twist()
    twistBackward.linear.x = -0.05
    return twistBackward

def twist_right():
    twistRight = Twist()
    twistRight.angular.z = -3.14
    return twistRight

def twist_left():
    twistLeft = Twist()
    twistLeft.angular.z = 3.14

FORWARD = twist_forward()
BACKWARD = twist_backward()
RIGHT = twist_right()
LEFT = twist_left()

class MoveUtil:

    def __init__(self, myMap):
        self._pub = rospy.Publisher(NAV_TOPIC, Twist)
        self._map = myMap

    def _twist_direction(self, direction):
        self._pub.publish(direction)
        time.sleep(0.5)

    def forwardThenTurn(self, pose):
        # Move forward so we have space to turn
        self._pub.publish(FORWARD)
        time.sleep(0.5)
        self._map.go_to_marker(pose, 60)

    def goToPose(self, pose, timeout):
        self._map.go_to_marker(pose, timeout)

