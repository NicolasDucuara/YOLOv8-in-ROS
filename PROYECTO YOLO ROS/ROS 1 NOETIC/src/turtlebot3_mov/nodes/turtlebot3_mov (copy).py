#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import Inference

class Detection():
    def __init__(self):
        self.init_scan_state = False 
        self.init_yolo_state = False
        self.stop = 1
        self.scan_ranges = []

        self.publisher_control = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.subscriber_laser = rospy.Subscriber(
            'scan',
            LaserScan,
            self.laser_callback,
            queue_size=1)
        
        self.subscriber_yolo = rospy.Subscriber(
            '/Yolov8_Inference',
            Inference,
            self.yolo_callback,
            queue_size=10)
        
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
            self.person_distance()
            self.signal_distance()


    def stoptb3(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_control.publish(twist)


    def person_detect(self, distance):
        object = self.class_name
        if 0.0 < distance <= 0.6:
            if object == 'person':
                rospy.loginfo('%s ' + 'to' + ' %f ' + 'meters',self.class_name, distance)
                self.stoptb3()
                #rospy.loginfo('Stopped')
                self.stop = 1


    def signal_detect(self, distance):
        object = self.class_name
        if 0.0 < distance <= 0.8:
            if object == 'stop sign':
                rospy.loginfo('%s ' + 'to' + ' %f ' + 'meters',self.class_name, distance)
                if self.stop == 1:
                    time_init = time.time()
                    while True:
                        time_end = time.time()
                        self.person_distance()
                        if time_end - time_init >= 5:
                            break
                    while True:
                        self.stoptb3()
                        time_end = time.time()
                        #rospy.loginfo('Stopped')
                        if time_end - time_init >= 11:
                            break
                    self.stop = 0
            else:
                self.stop = 1


    def person_distance(self):
        #range_scan = [14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,359,358,357,356,355,
        #                354,353,352,351,350]
        range_scan = [11,10,9,8,7,6,5,4,3,2,1,0,359,358,357,356,355,
                        354,353,352,351,350,349,348,347]
        for i in range_scan:
            distance = self.scan_ranges[i]
            self.person_detect(distance)


    def signal_distance(self):
        distance = 3.5
        angle = 0
        central_point = (self.right + self.left)/2
        if 0 <= central_point <= 159:   #240
        #if 0 <= central_point <= 259:   #360
        #if 0 <= central_point <= 319:   #480
            angle = -(round((central_point * (25/160))-25))   #240
            #angle = -(round((central_point * (25/260))-25))   #360
            #angle = -(round((central_point * (25/320))-25))    #480
            if 3 <= angle <= 356:
                range_scan = [angle-3, angle-2, angle-1, angle, angle+1, angle+2, angle+3]
                for i in range_scan:
                    distance = self.scan_ranges[i]
                    self.signal_detect(distance)
        elif 160 <= central_point <= 319:  #240
        #elif 260 <= central_point <= 519:  #360
        #elif 320 <= central_point <= 639:  #480 
            angle = -(round((central_point * (25/160))-385))  #240
            #angle = -(round((central_point * (25/260))-385))   #360
            #angle = -(round((central_point * (25/320))-385))   #480
            if 3 <= angle <= 356:
                range_scan = [angle-3, angle-2, angle-1, angle, angle+1, angle+2, angle+3]
                for i in range_scan:
                    distance = self.scan_ranges[i]
                    self.signal_detect(distance)

if __name__ == '__main__':
    rospy.init_node('turtlebot3_mov')
    detection = Detection()
    rospy.spin()

