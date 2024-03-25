#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import Inference
from std_msgs.msg import Bool

twist = Twist()
LED_P = Bool()
LED_S = Bool()

class Detection():

    def __init__(self):
        self.init_scan_state = False 
        self.init_yolo_state = False
        self.stop = 0
        self.scan_ranges = []

        self.publisher_control = rospy.Publisher('cmd_vel', Twist, queue_size=3)

        self.led_person = rospy.Publisher('/led_person', Bool, queue_size=5)
        self.led_signal = rospy.Publisher('/led_signal', Bool, queue_size=5)


        self.subscriber_laser = rospy.Subscriber(
            'scan',
            LaserScan,
            self.laser_callback,
            queue_size=5)
        
        self.subscriber_yolo = rospy.Subscriber(
            '/Yolov8_Inference',
            Inference,
            self.yolo_callback,
            queue_size=1)
        
        self.update_timer = rospy.Timer(rospy.Duration(0.001), self.callback_update)

        

    def laser_callback(self, data):
        self.scan_ranges = data.ranges
        self.init_scan_state = True

    def yolo_callback(self, data):

        for r in data.inference:
            self.class_name = r.class_name
            self.top = r.top
            self.left = r.left
            self.bottom = r.bottom
            self.right = r.right
        self.init_yolo_state = True


    def callback_update(self, *args):
        if self.init_yolo_state is True and self.init_scan_state is True:
            self.person_detect()
            self.signal_detect()


    def stoptb3(self):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_control.publish(twist)

    def person_detect(self):
        object = self.class_name
        if object == 'person':
            range_scan = [15,10,5,0,359,354,349,344]
            for i in range_scan:
                distance = self.scan_ranges[i]
                if 0.0 < distance <= 0.55:
                    self.stoptb3()
                    self.led_person.publish(Bool(True))
                    #rospy.loginfo('%s ' + 'to' + ' %f ' + 'meters',self.class_name, distance)
                    #rospy.loginfo('stopped')
                else:
                    self.led_person.publish(Bool(False))

    def signal_detect(self):
        object = self.class_name
        if object == 'stop sign':
            distance = self.signal_distance()
            if 0.0 < distance <= 0.65:
                rospy.loginfo('%s ' + 'to' + ' %f ' + 'meters',object, distance)
                self.stop = 1
                time_init = time.time()
                while True:
                    time_end = time.time()
                    self.person_detect()
                    if time_end - time_init >= 4:
                        break
                self.led_signal.publish(Bool(True))
                while True:
                    self.stoptb3()
                    time_end = time.time()
                    if time_end - time_init >= 8:
                        self.stop = 0
                        self.led_signal.publish(Bool(False))
                        break
            

    def signal_distance(self):
        real_width = 0.019
        measured_distance = 0.5
        width_pixels_stop = 125  #250  #125
        width_pixels = self.bottom - self.top
        focal_lenth = (width_pixels_stop * measured_distance) / real_width
        distance = (real_width * focal_lenth) / width_pixels

        return distance


if __name__ == '__main__':
    
    rospy.init_node('turtlebot3_mov')
    detection = Detection()

    rospy.spin()

