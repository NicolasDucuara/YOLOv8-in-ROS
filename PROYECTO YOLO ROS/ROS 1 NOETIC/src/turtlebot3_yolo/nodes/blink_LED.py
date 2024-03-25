#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('turtlebot3_gpio_control')

import sys
import rospy

from std_msgs.msg import Bool

import numpy as np

import RPi.GPIO as GPIO
import time

class blink_led_node:
    def __init__(self):
        """ init RPi GPIO """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)

        """  Initializing your ROS Node """
        rospy.init_node('blink_led_node', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        """ Subscribe to the Bool topic """
        self.ledPerson_sub = rospy.Subscriber("/led_person", Bool, self.callback_person)
        self.ledSignal_sub = rospy.Subscriber("/led_signal", Bool, self.callback_signal)


    def callback_person(self,data):
        if (data.data == True):
            rospy.loginfo("LED Person ON!")
            GPIO.output(11, GPIO.HIGH)

        elif (data.data == False):
            rospy.logwarn("LED Person OFF!")
            GPIO.output(11, GPIO.LOW)


    def callback_signal(self,data):
        if (data.data == True):
            rospy.loginfo("LED Signal ON!")
            GPIO.output(13, GPIO.HIGH)

        elif (data.data == False):
            rospy.logwarn("LED Signal OFF!")
            GPIO.output(13, GPIO.LOW)

    def shutdown(self):
        try:
            rospy.loginfo("Blink LED node [OFFLINE]...")

        finally:
            GPIO.cleanup()

def usage():
    print("%s" % sys.argv[0],)

def main(args):
    vn = blink_led_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Blink LED node [OFFLINE]...")


if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Blink LED node [ONLINE]...")
        main(sys.argv)