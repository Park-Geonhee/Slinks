#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import math
import random
import tf

from cv_bridge import CvBridgeError
from sklearn import linear_model

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped, Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import warnings


params_cam ={
        "ENGINE": "UNITY",
        "TYPE": "JPG",
        "WIDTH": 640,
        "HEIGHT": 480,
        "FOV": 90,
        "X": 3.50,
        "Y": 0,
        "Z": 0.50,
        "YAW": 0,
        "PITCH": 0.0,
        "ROLL": 0
}

def grayscale(img): # 흑백이미지로 변환
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img, low_threshold, high_threshold): # Canny 알고리즘
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size): # 가우시안 필터
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, crop_pts, color3=(255,255,255), color1=255): # ROI 셋팅

    mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
    if len(img.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillPoly(mask, crop_pts, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def draw_fit_line(img, lines, color=[0, 0, 255], thickness=10): # 대표선 그리기
    try:
        cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), color, thickness)
    except:
        pass
    return img

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    #draw_lines(line_img, lines)

    return lines

def weighted_img(img, initial_img, α=1, β=1., λ=0.): # 두 이미지 operlap 하기
    return cv2.addWeighted(initial_img, α, img, β, λ)

def get_fitline(img, f_lines): # 대표선 구하기   
    try:
        f_lines= f_lines.reshape(1,-1,2)[0]
        print(f_lines)
        output = cv2.fitLine(f_lines,cv2.DIST_L2,0, 0.01, 0.01)
        

        vx, vy, x, y = output[0], output[1], output[2], output[3]
        x1, y1 = int(((img.shape[0]-1)-y)/vy*vx + x) , img.shape[0]-1
        x2, y2 = int(((img.shape[0]/2+100)-y)/vy*vx + x) , int(img.shape[0]/2+100)
        
        result = [x1,y1,x2,y2]
        return result
    except:
        return []


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rate = rospy.Rate(10)
        self.img_bgr = None
        self.img_lane = None
        self.edges = None
        self.is_status = False

        while not rospy.is_shutdown():
            if self.img_bgr is not None and self.is_status == True:
                height,width = self.img_bgr.shape[0],self.img_bgr.shape[1]
                gray_img = grayscale(self.img_bgr) # 흑백이미지로 변환
                cv2.imwrite('gray.jpg',gray_img)
                blur_img = gaussian_blur(gray_img, 3) # Blur 효과
                cv2.imwrite('gaussian.jpg',blur_img)
                canny_img = canny(blur_img, 70, 210) # Canny edge 알고리즘
                cv2.imwrite('canny.jpg',canny_img)
                crop_pts = np.array([[[0, 480], [0, 350], [280, 200], [360, 200], [640, 350], [640, 480]]])
                ROI_img = region_of_interest(canny_img, crop_pts) # ROI 설정
                cv2.imwrite('ROI.jpg',ROI_img)

                try:
                    line_arr = hough_lines(ROI_img, 1, 1 * np.pi/180, 30, 5, 10) # 허프 변환
                    line_arr = np.squeeze(line_arr)
                    slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi
                except:
                    cv2.imshow('result',self.img_bgr) # 결과 이미지 출력
                    cv2.waitKey(1)
                    continue

                # 수평 기울기 제한
                line_arr = line_arr[np.abs(slope_degree)<181]
                slope_degree = slope_degree[np.abs(slope_degree)<181]

                # 수직 기울기 제한
                line_arr = line_arr[np.abs(slope_degree)>170]
                slope_degree = slope_degree[np.abs(slope_degree)>170]


                L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
                temp = np.zeros((self.img_bgr.shape[0], self.img_bgr.shape[1], 3), dtype=np.uint8)
                L_lines, R_lines = L_lines[:,None], R_lines[:,None]

                left_fit_line = get_fitline(self.img_bgr,L_lines)

                # 대표선 그리기
                temp = draw_fit_line(temp, left_fit_line)

                result = weighted_img(temp, self.img_bgr) # 원본 이미지에 검출된 선 overlap
                cv2.imshow('result',result) # 결과 이미지 출력
                cv2.waitKey(1)
            rate.sleep()

    def odom_callback(self, msg):  ## Vehicl Status Subscriber
        self.status_msg = msg
        self.is_status = True

    def ego_callback(self,msg):
        print(msg)

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)


if __name__=='__main__':
    try:
        rospy.init_node('abc',anonymous=True)
        i=IMGParser()
    except rospy.ROSInternalException as e:
        pass