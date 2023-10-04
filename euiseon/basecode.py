#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import torch
import cv2
import rospy
import numpy as np
import time
import os
from sensor_msgs.msg import CompressedImage

# Model
# model = torch.hub.load("ultralytics/yolov5", "yolov5n")  # or yolov5n - yolov5x6, custom

# Images
# img = "https://ultralytics.com/images/zidane.jpg"  # or file, Path, PIL, OpenCV, numpy, list


# Inference
# results = model(img)

# Results
# results.print()
# results.show()
# results.save()  # or .show()
Path = os.path.dirname(os.path.realpath(__file__)) + "/yolov5/"
class YoloObject :
    def __init__(self):
        self.model = torch.hub.load(Path,'custom','yolov5n.pt', source='local')  # or yolov5n - yolov5x6, custom
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.image = None
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.image is not None:
                results =self.model(self.image)
                results.show()
                time.sleep(1000)
                # print(results.pandas().xyxy[0]["xmin"][0])
                # print(results.pandas().xyxy[0]["xmax"])                    
            rate.sleep()

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image= cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # results = self.model(self.img)
        # results.show()
        # results.print()
        # print(results.pandas().xyxy[0])
        
        # cv2.imshow("image", self.img)
        # cv2.waitKey(1)

if __name__=="__main__":
    rospy.init_node("imageprocess",anonymous=True)
    test = YoloObject()
    rospy.spin() 