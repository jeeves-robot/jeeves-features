#!/usr/bin/env python

import rospy
import time

from map_util import MapUtil

IDLE_STATE = 0
WAIT_PACKAGE_STATE = 1
TO_ROOM = 2
AT_ROOM = 3
TO_FRONT_DOOR = 4
TO_FRONT_DESK = 5

WAIT_PACKAGE_POSE = "wait_package_pose"
HOME_POSE = "home"

QRCODE_TOPIC = "/jeeves_qr_code"

MAP_FILE = "locations.csv"

class RobotFSM:
    # Passed as listener to topic qrcode
    # Gets a Order object
    def validateQRCode(self, qrcode):
        # see if location is one we know about
        if self._state == IDLE_STATE and self._map.contains(qrcode.location):
            self._order = qrcode
            # Turn around
            self._state = WAIT_PACKAGE_STATE
            self._move_util.forwardThenTurn(self, WAIT_PACKAGE_POSE)
            # Wait for package (in future this will involve scale readings)
            time.sleep(20)
            self.packageReceived()
        else:
            # TODO: Tell delivery person QRCode is invalid
            pass

    def packageReceived(self):
        if self._state == WAIT_PACKAGE_STATE:
            self._state = TO_ROOM
            # Note, this will hang until we reach the location or give up
            # TODO: Handle failure
            if self._move_util.goToPose(self._order.location, 300):
                self.reachedDoor()

    def reachedDoor(self):
        if self._state == TO_ROOM:
            self._state = AT_ROOM
            # TODO: Send message
            # Wait for package to be taken
            # In future this will involve scale readings
            # and handle case where it is not picked up
            time.sleep(60)
            packageDelivered(self)

    def packageDelivered(self):
        if self._state == AT_ROOM:
            self._state = TO_FRONT_DOOR
            if self._move_util.goToPose(HOME_POSE, 300):
                self._state = IDLE_STATE

    def __init__(self, floorMap):
        rospy.init_node('jeeves_main')
        self._state = IDLE_STATE
        self._map = floorMap
        self._move_util = MoveUtil(floorMap)
        self._order = None

        # Set up listener for QRCode message
        self._qrcode_subscriber = rospy.Subscriber(QRCODE_TOPIC,
                Order, self.validateQRCode)

if __name__ == '__main__':
    floorMap = MapUtil(MAP_FILE)
    robotFSM = RobotFSM(floorMap)
    rospy.spin()

