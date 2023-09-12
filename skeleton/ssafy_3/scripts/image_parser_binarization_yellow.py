#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser binarization Node 는 시뮬레이터에서 송신하는 Camera 센서 정보를 받아 실시간으로 출력하는 예제입니다.
# 출력시 hsv 특정 영역의 색상 범위를 지정하여 원하는 색상의 영역만 특정하여 출력합니다.

# 노드 실행 순서 
# 1. HSV 색상 영역 지정
# 2. 특정 영역의 색상 검출
# 3. 이미지 출력

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
        # 이번 예제에서는 노란색 영역을 검출합니다.
        lower_ylane = np.array([0,70,120])# ([0,60,100])
        upper_ylane = np.array([40,195,230])# ([40,175,255])

        img_ylane = cv2.inRange(img_hsv,lower_ylane,upper_ylane)

        img_result = cv2.bitwise_and(img_hsv,img_hsv,mask = img_ylane)
        
        img_concat = np.concatenate([img_bgr,img_result],axis=1)

        #TODO: (3)
        cv2.imshow("Binarize Yellow Image", img_concat)
        cv2.waitKey(1) 


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 