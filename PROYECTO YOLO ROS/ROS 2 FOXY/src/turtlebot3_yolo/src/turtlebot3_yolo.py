#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from turtlebot3_msgs.msg import InferenceResult
from turtlebot3_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')   #Constructor

        self.model = YOLO('~/turtlebot3_ws_foxy/src/turtlebot3_yolo/src/yolov8n.pt')
        
        self.yolov8_inference = Yolov8Inference()
        self.inference_result = InferenceResult()
        
        qos = QoSProfile(depth=10)
        
        self.subscription = self.create_subscription(    #Se crea un subscriptor que recibe las imagenes de ROS
            Image,
            '/camera/image_raw',
            #'/image_raw',
            self.camera_callback,
            qos_profile=qos_profile_sensor_data)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 10)         #Publicador en matriz
        self.img_pub = self.create_publisher(Image, "/inference_result_image", 10)                #Publicador en imagen


    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")       #Se convierte de ROS Images a cv2 Images
        results = self.model.predict(img)              #Se ingresa la imagen con formato cv2 a YOLOv8 y se guarda en resultado

        self.inference_result.class_name = ''
        self.inference_result.left = 0
        self.inference_result.top = 0
        self.inference_result.right = 0
        self.inference_result.bottom = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]        # obtiene coordenadas en (top, left, bottom, right)
                c = box.cls
                class_name = self.model.names[int(c)]
                if class_name == 'stop sign' or class_name == 'person':
                    self.inference_result.class_name = class_name
                    self.inference_result.left = int(b[0])
                    self.inference_result.top = int(b[1])
                    self.inference_result.right = int(b[2])
                    self.inference_result.bottom = int(b[3])
                
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.img_pub.publish(img_msg)

        self.yolov8_inference.yolov8_inference.append(self.inference_result)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()
