#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        #TODO: (1)
        # 이번 예제에서는 bgr이미지를 HSV 이미지로 변환합니다.
        img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
        # bgr 이미지와 hsv 이미지를 동시에 출력하기 위해 np.concatenate 함수를 사용해 이미지를 합칩니다.
        img_concat = np.concatenate([img_bgr,img_hsv],axis=1)

        cv2.imshow("HSV IMAGE",img_concat)
        cv2.waitKey(1) 

if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 