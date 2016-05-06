#!/usr/bin/env python

import rospy
from move_util import NAV_TOPIC, twist_forward, twist_direction
from geometry_msgs.msg import Twist
from jeeves.msg import Order

nav_publisher = rospy.Publisher(NAV_TOPIC, Twist)

busy = False

def turn_around():
    busy = True
    # Go forward
    twist_direction(nav_publisher, twist_forward())
    # Do a 180
    twist_direction(nav_publisher, twist_right())
    twist_direction(nav_publisher, twist_right())
    # Go forward
    twist_direction(nav_publisher, twist_forward())

    time.sleep(0.2)
    busy = False # TODO actually waiting for package

def qr_decoded_cb(event):
    if not busy:
        turn_around()

if __name__ == "__main__":
    rospy.init_node('camera_listener')

    # Subscribe to topic to get notifications when we decode a QR code
    sub = rospy.Subscriber('/jeeves_qr_code', Order, qr_decoded_cb)
    rospy.spin()
