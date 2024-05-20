#!/usr/bin/env python3


'''
 Class names:
0- Blue-ball
1- Purple-ball
2- Red-Ball
3- silo

'''

from rclpy.node import Node
import rclpy
import cv_bridge
import cv2
from time import time
import os
import numpy as np

from ultralytics import YOLO
from ultralytics import utils

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from r2_interfaces.msg import YoloResults
from r2_interfaces.msg import Xywh
from r2_interfaces.msg import XyXy


class YoloObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('yolo_object_tracking_node')
        self.get_logger().info("Cam Object Detect Node has been started")

        #Loads the current script directory
        current_script_dir = os.path.dirname(os.path.realpath(__file__))

        #Loads the model
        model_path = os.path.join(current_script_dir, 'weights/model_sim.pt')
        self.model = YOLO(model_path)
        
        # Set the IOU and Confidence threshold
        self.iou = 0.5
        self.conf = 0.0

        # Subscribe to the camera image topic
        self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish the annotated image
        self.publisher = self.create_publisher(
                                Image,
                                'camera/image_raw/annotated',
                                10
                            )
        # Publish the YOLO results
        self.yolo_result_publisher = self.create_publisher(
                                YoloResults,
                                'yolo_results',
                                10
                            )



    def image_callback(self, msg):
        """
        Callback function for processing the received image message.

        Args:
            msg: The image message received.

        Converts the image message to a cv2 image and processes it using the YOLO model.
        
        """
        self.get_logger().info("Image received")
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) 

        # cv2.imshow('image', img)
        # cv2.waitKey(1)

        self.yolo_object_detect(img)


    def yolo_object_detect(self, img):
        
        '''
        Function to detect objects in the image using the YOLO model.
        
        Plot the annotated image and publish the results.
        
        Results are published in the YoloResults message format.
        '''
        
        # Perform object detection and stores the results
        results = self.model.track(img, persist=True, conf=self.conf, iou=self.iou)
        
        # Create a YoloResults message to store the results
        yolo_result_msg = YoloResults()
        
        # Loop through the results and store the bounding box and stores the results in the YoloResults message
        for result in results:
            
            print(result.boxes)
            
            xywh_list = result.boxes.xywh.tolist() # center_x, center_y, width, height
            
            # print(xywh_list)
            
            for xywh in xywh_list:
                xywh_msg = Xywh()
                xywh_msg.center_x = xywh[0]
                xywh_msg.center_y = xywh[1]
                xywh_msg.width= xywh[2]
                xywh_msg.height = xywh[3]
                
                yolo_result_msg.xywh.append(xywh_msg)   #
                
            xyxy_list = result.boxes.xyxy.tolist() # top_left_x, top_left_y, bottom_right_x, bottom_right_y
            # print(xyxy_list)
            
            for xyxy in xyxy_list:
                xyxy_msg = XyXy()
                xyxy_msg.tl_x = xyxy[0]
                xyxy_msg.tl_y = xyxy[1]
                xyxy_msg.br_x = xyxy[2]
                xyxy_msg.br_y = xyxy[3]
                
                yolo_result_msg.xyxy.append(xyxy_msg)
                
                
        # Class ids: 0- Blue-ball, 1- Purple-ball, 2- Red-Ball, 3- silo        
        cls_list = [int(cls) for cls in result.boxes.cls.tolist()]
        yolo_result_msg.class_ids.extend(cls_list)
        
        # Confidence values
        conf_list = result.boxes.conf.tolist()
        yolo_result_msg.confidence.extend(conf_list)
        
        # Tracking ids
        if result.boxes.id is not None:
            ids_list = result.boxes.id.tolist()
            yolo_result_msg.tracking_id.extend(ids_list)
        

        # Plot the annotated image
        annotated_frame = results[0].plot()

        cv2.imshow('YOLOv8 Tracking', annotated_frame)
        cv2.waitKey(1)

        # Convert the annotated image to a ROS message
        bridge = cv_bridge.CvBridge()
        annotated_frame_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        
        
        # Publish the results        
        self.yolo_result_publisher.publish(yolo_result_msg)
        self.publisher.publish(annotated_frame_msg)
        
    
def main(args=None):
    rclpy.init(args=args)
    yolo_object_tracking_node = YoloObjectTrackingNode()
    rclpy.spin(yolo_object_tracking_node)
    yolo_object_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


