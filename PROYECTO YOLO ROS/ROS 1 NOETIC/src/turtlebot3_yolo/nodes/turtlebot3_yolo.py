#!/usr/bin/env python3

from ultralytics import YOLO
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from turtlebot3_msgs.msg import InferenceResult
from turtlebot3_msgs.msg import Inference
from time import sleep

bridge = CvBridge()

class Yolo_detection():
    def __init__(self):
        self.model = YOLO('yolov8n.pt')
        self.inference = Inference()
        self.inference_result = InferenceResult()

        self.subscription = rospy.Subscriber(
            #'/raspicam_node/image',
            '/usb_cam/image_raw',
            #'/image_raw',
            Image,
            self.camera_callback,
            queue_size=10)
        
        self.yolov8_pub = rospy.Publisher('/Yolov8_Inference',Inference,queue_size=1)
        self.img_pub = rospy.Publisher('/inference_result_image',Image,queue_size=1)  

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        self.results = self.model.predict(img)
        
        self.inference_result.class_name = ''
        self.inference_result.left = 0
        self.inference_result.top = 0
        self.inference_result.right = 0
        self.inference_result.bottom = 0

        for r in self.results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0] 
                c = box.cls
                class_name = self.model.names[int(c)]
                if class_name  == 'stop sign' or class_name == 'person':
                    self.inference_result.class_name = self.model.names[int(c)]
                    self.inference_result.left = int(b[0])
                    self.inference_result.top = int(b[1])
                    self.inference_result.right = int(b[2])
                    self.inference_result.bottom = int(b[3])

        annotated_frame = self.results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.img_pub.publish(img_msg)
        self.inference.inference.append(self.inference_result)
        self.yolov8_pub.publish(self.inference)
        self.inference.inference.clear()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_yolo')
    yolo_detection = Yolo_detection()
    rospy.spin()
    
