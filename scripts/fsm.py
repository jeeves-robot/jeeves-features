#!/usr/bin/env python

import sys
import rospy
import time
import threading
from firebase import firebase
from map_util import MapUtil as MapUtil
from move_util import MoveUtil as MoveUtil
from jeeves.msg import *

IDLE_STATE = 0
WAIT_PACKAGE_STATE = 1
TO_ROOM = 2
AT_ROOM = 3
TO_FRONT_DOOR = 4
TO_FRONT_DESK = 5
AT_FRONT_DESK = 6

STATES = {
        0 : 'Idle',
        1 : 'Wait for package',
        2 : 'To room',
        3 : 'At room',
        4 : 'To front door',
        5 : 'To front desk',
        6 : 'At front desk',
        }

WAIT_PACKAGE_POSE = "wait_package_pose"
HOME_POSE = "home"
FRONT_DESK = "front_desk"

QRCODE_TOPIC = "/jeeves_qr_code"
FIREBASE_URL = "https://jeeves-server.firebaseio.com"

WAIT_TIME = 1
TO_DESTINATION_TIME = 300

class RobotFSM:

    def changeToState(self, state):
        # Assumes lock is already acquired
        print "Changing to state", STATES[state]
        self._state = state

    # Passed as listener to topic qrcode
    # Gets a Order object
    def validateQRCode(self, qrcode):
        # see if location is one we know about
        print "Callback called"

        self._state_lock.acquire()
        if self._state == IDLE_STATE and self._map.contains(qrcode.location):
            self._qrcode_subscriber.unregister()
            self._order = qrcode
            # Turn around
            self.changeToState(WAIT_PACKAGE_STATE)
            self._move_util.forwardThenTurn(WAIT_PACKAGE_POSE)
            # Wait for package (in future this will involve scale readings)
            time.sleep(WAIT_TIME)
            self._state_lock.release()
            self.packageReceived()
        else:
            self._state_lock.release()
            # TODO: Tell delivery person QRCode is invalid

    def packageReceived(self):
        self._state_lock.acquire()
        if self._state == WAIT_PACKAGE_STATE:
            self.changeToState(TO_ROOM)
            # Note, this will hang until we reach the location or give up
            # TODO: Handle failure
            if self._move_util.goToPose(self._order.location, TO_DESTINATION_TIME):
                self._state_lock.release()
                print('calling reached door')
                self.reachedDoor()
            else:
                self.changeToState(TO_FRONT_DESK)
                self._state_lock.release()
                self.toFrontDesk()
        else:
            self._state_lock.release()

    def reachedDoor(self):
        self._state_lock.acquire()
        print(self._state)
        if self._state == TO_ROOM:
            self.changeToState(AT_ROOM)
            # Send notification
            message = self._order.name + " , your delivery is at your door!"
            notif = { 'to' : self._order.phone_number, 'body' : message}
            self._firebaseRef.post('/notifs', notif)
            # Wait for package to be taken
            # In future this will involve scale readings
            # and handle case where it is not picked up
            # TODO: go to front desk if package not picked up
            time.sleep(WAIT_TIME)
            self._state_lock.release()
            self.packageDelivered()
        else:
            self._state_lock.release()

    def toFrontDesk(self):
        self._state_lock.acquire()
        if self._state == TO_FRONT_DESK:
            if self._move_util.goToPose(FRONT_DESK, TO_DESTINATION_TIME):
                self.changeToState(AT_FRONT_DESK)
                time.sleep(WAIT_TIME)
        self._state_lock.release()
        self.packageDelivered()

    def packageDelivered(self):
        self._state_lock.acquire()
        if self._state == AT_ROOM or self._state == AT_FRONT_DESK:
            self.changeToState(TO_FRONT_DOOR)
            print "Going to home"
            if self._move_util.goToPose(HOME_POSE, TO_DESTINATION_TIME):
                self.changeToState(IDLE_STATE)
                self._qrcode_subscriber = self.createSubscriber()
                self._state_lock.release()
            else:
                self.changeToState(TO_FRONT_DESK)
                self._state_lock.release()
                self.toFrontDesk()
        else:
            self._state_lock.release()

    def createSubscriber(self):
        return rospy.Subscriber(QRCODE_TOPIC, Order, self.validateQRCode)

    def __init__(self, locationsFile):
        rospy.init_node('jeeves_main')
        self._state_lock = threading.Lock();
        self._state_lock.acquire()
        self.changeToState(IDLE_STATE)
        self._map = MapUtil(locationsFile)
        self._move_util = MoveUtil(self._map)
        self._order = None
        self._state_lock.release()

        # Set up listener for QRCode message
        self._qrcode_subscriber = self.createSubscriber()
        # Set up firebase ref for notifications
        self._firebaseRef = firebase.FirebaseApplication(FIREBASE_URL)

if __name__ == '__main__':
    if len(sys.argv) < 2:
      print "Pass filename of locations file as argument to command line program."
      print "Defaulting to markers.csv"
      locations_file = "markers.csv"
    else:
      locations_file = sys.argv[1]
    robotFSM = RobotFSM(locations_file)
    rospy.spin()

