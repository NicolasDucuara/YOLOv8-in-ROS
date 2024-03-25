#!/usr/bin/python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.msg import Yolov8Inference

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        self.init_scan_state = False 
        self.init_yolo_state = False
        
        self.stop = 1
        self.scan_ranges = []

        qos = QoSProfile(depth=10)

        #Publisher
        self.publisher_control = self.create_publisher(Twist, 'cmd_vel', qos)

        #Subscribers
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile=qos_profile_sensor_data)

        self.subscriber_yolo = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            qos_profile=qos_profile_sensor_data)

        #Timer
        self.update_timer = self.create_timer(0.01, self.callback_update)
        
        self.get_logger().info("Turtlebot3 YOLO")

    def laser_callback(self, data):
        self.scan_ranges = data.ranges
        self.init_scan_state = True

    def yolo_callback(self, data):

        for r in data.yolov8_inference:
            self.class_name = r.class_name
            self.top = r.top
            self.left = r.left
            self.bottom = r.bottom
            self.right = r.right
        self.init_yolo_state = True

    def callback_update(self):
        if self.init_yolo_state is True and self.init_scan_state is True:
            self.object_distance()

        
    def stoptb3(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_control.publish(twist)


    def object_distance(self):
        range_scan = [20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,359,358,357,356,355,
                        354,353,352,351,350,349,348,347,346,345,344,343,342,341,340,339]
        for i in range_scan:
            distance = self.scan_ranges[i]
            self.object_detect(distance)

        
    def object_detect(self, distance):
        object = self.class_name
        if 0.0 < distance <= 0.45:
            if object == 'person':
                    self.get_logger().info(f"{self.class_name} to {distance} meters")
                    self.stoptb3()
        elif 0.0 < distance <= 0.6:
            if object == 'stop sign':
                self.get_logger().info(f"{self.class_name} to {distance} meters")
                if self.stop == 1:
                    time_init = time.time()
                    while True:
                        time_end = time.time()
                        if time_end - time_init >=5:
                            break
                    while True:
                        self.stoptb3()
                        time_end = time.time()
                        if time_end - time_init >= 5:
                            break
                    self.stop = 0
            else:
                self.stop = 1



if __name__ == '__main__':
    rclpy.init(args=None)
    detection = Detection()
    rclpy.spin(detection)
    detection.destroy_node()
    rclpy.shutdown()
