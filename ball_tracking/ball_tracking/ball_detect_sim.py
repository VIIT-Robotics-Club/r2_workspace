#!/usr/bin/env python3


# /camera/image_raw
import rclpy
import cv_bridge
import cv2
from ultralytics import YOLO
from rclpy.node import Node
from time import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import os

class CamObjectDetectNode(Node):
    def __init__(self):
        super().__init__('cam_object_detect_node')
        self.get_logger().info("Cam Object Detect Node has been started")

        current_script_dir = os.path.dirname(os.path.realpath(__file__))

        model_path = os.path.join(current_script_dir, 'weights/model_sim.pt')

        self.model = YOLO(model_path)
        self.iou = 0.5
        self.conf = 0.0

       
        
        self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        
        ### USE THIS WHEN USING COMPRESSED IMAGE
        # self.create_subscription(
        #     CompressedImage,
        #     'camera/image_raw/compressed',
        #     self.compressed_image_callback,
        #     10
        # )
 

        ### USE THIS WHEN USING UNCOMPRESSED(Compressed -> uncompressed) from image_pipeline IMAGE
        # self.create_subscription(
        #     Image,
        #     'camera/image_raw/uncompressed',
        #     self.image_uncompressed_callback,
        #     10
        # )

        self.publisher = self.create_publisher(
                                Image,
                                'camera/image_raw/annotated',
                                10
                            )


    def image_callback(self, msg):
        self.get_logger().info("Image received")
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) 

        cv2.imshow('image', img)
        cv2.waitKey(1)
        
        self.yolo_object_detect(img)



    def compressed_image_callback(self, msg):
        self.get_logger().info("Compressed Image received")
        bridge = cv_bridge.CvBridge()
        img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow('compressed image', img)
        cv2.waitKey(1)

        
    def image_uncompressed_callback(self, msg):
        self.get_logger().info("Uncompressed Image received")
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow('uncompressed image', img)
        cv2.waitKey(1)


    def yolo_object_detect(self, img):
        results = self.model.track(img, persist=True, conf=self.conf, iou=self.iou)

        annotated_frame = results[0].plot()

        cv2.imshow('YOLOv8 Tracking', annotated_frame)
        cv2.waitKey(1)

        bridge = cv_bridge.CvBridge()
        annotated_frame_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        self.publisher.publish(annotated_frame_msg)
        
    
    


def main(args=None):
    rclpy.init(args=args)
    cam_object_detect_node = CamObjectDetectNode()
    rclpy.spin(cam_object_detect_node)
    cam_object_detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


