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
        self.stop = 0
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
            self.person_detect()
            self.signal_detect()


    def stoptb3(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_control.publish(twist)

    def person_detect(self):
        object = self.class_name
        if object == 'person':
            range_scan = [15,12,9,6,3,0,357,354,351,348,345]
            for i in range_scan:
                distance = self.scan_ranges[i]
                if 0.0 < distance <= 0.5:
                    self.stoptb3()
                    rospy.loginfo('%s ' + 'to' + ' %f ' + 'meters',self.class_name, distance)
            #rospy.loginfo('Stopped')
            self.stop = 0


    def signal_detect(self):
        object = self.class_name
        if object == 'stop sign':
            distance = self.signal_distance()
            if 0.0 < distance <= 0.75:
                rospy.loginfo('%s ' + 'to' + ' %f ' + 'meters',object, distance)
                self.stop = 1

        if 0.75 <= self.scan_ranges[0] <= 0.8 and self.stop == 1:
            time_init = time.time()
            while True:
                self.stoptb3()
                self.person_detect()
                time_end = time.time()
                if time_end - time_init >= 5:
                    rospy.loginfo('stopped')
                    self.stop = 0
                    break

    def signal_distance(self):
        distance = 3.5
        angle = 0
        central_point = (self.right + self.left)/2
        if 0 <= central_point <= 159:   #240
        #if 0 <= central_point <= 319:   #480
            angle = -(round((central_point * (25/160))-25))   #240
            #angle = -(round((central_point * (25/320))-25))    #480
            if 5 <= angle <= 354:
                range_scan = [angle-5, angle-3, angle-1, angle, angle+1, angle+3, angle+5]
                for i in range_scan:
                    distance = self.scan_ranges[i]
                    return distance
        elif 160 <= central_point <= 319:  #240
        #elif 320 <= central_point <= 639:  #480 
            angle = -(round((central_point * (25/160))-385))  #240
            #angle = -(round((central_point * (25/320))-385))   #480
            if 5 <= angle <= 354:
                range_scan = [angle-5, angle-3, angle-1, angle, angle+1, angle+3, angle+5]
                for i in range_scan:
                    distance = self.scan_ranges[i]
                    return distance
        return distance

if __name__ == '__main__':
    rospy.init_node('turtlebot3_mov')
    detection = Detection()
    rospy.spin()

