#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

def warp_image(img, source_prop):
    
    image_size = (img.shape[1], img.shape[0])

    x = img.shape[1]
    y = img.shape[0]
    l_t = [280,240]
    l_b = [0,350]
    r_t = [360,240]
    r_b = [640,350]
    ## left top->letf bottom->right top -> right bottom
    source_points = np.float32([l_t,l_b,r_t,r_b])
    destination_points = np.float32([[0,0],[0,480],[640,0],[640,480]])

    perspective_transform = cv2.getPerspectiveTransform(source_points,destination_points)
    warped_img = cv2.warpPerspective(img, perspective_transform,image_size)

    ## draw move points 
    cv2.circle(img, tuple(l_t), 5, (255,0,0),-1)
    cv2.circle(img, tuple(l_b), 5, (0,255,0),-1)
    cv2.circle(img, tuple(r_t), 5, (0,0,255),-1)
    cv2.circle(img, tuple(r_b), 5, (0,0,0),-1)
    return warped_img


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

        self.source_prop = np.float32([0.7])

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_warp = warp_image(self.img_bgr, self.source_prop)
        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


def main():
    rospy.init_node('lane_birdview', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()

if __name__ == '__main__':
    main()
