#! /usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import torch
import numpy as np
import os

from cv_bridge import CvBridgeError
from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import GetTrafficLightStatus

lower_red = np.array([-10, 30, 50]) 
upper_red = np.array([10, 255, 255])

lower_green = np.array([50, 80, 80])
upper_green = np.array([90, 255, 255])

lower_yellow = np.array([11, 50, 50])
upper_yellow = np.array([30, 200, 200])

PATH = "/home/leesh/catkin_ws/src/ssafy_ad/S09P22A701/seunghyuk/traffic_light"
os.chdir(PATH)

def Check_Color(img, traffic_light):
    try:
        img = img[int(traffic_light["ymin"]):int(traffic_light["ymax"]),int(traffic_light["xmin"]):int(traffic_light["xmax"])]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cv2.imshow("od",img)
        cv2.waitKey(1)
    except:
        return 0
    
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    red_areas = cv2.bitwise_and(img, img, mask=red_mask)

    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    green_areas = cv2.bitwise_and(img, img, mask=green_mask)

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_areas = cv2.bitwise_and(img, img, mask=yellow_mask)

    flag=0
    if np.any(red_areas):
        flag+=1
    if np.any(green_areas):
        flag+=2
    if np.any(yellow_areas):
        flag+=4

    return flag

class Object_Detect:
    def __init__(self):
        self.model = torch.hub.load(PATH, 'custom', 'yolov5s.onnx',source='local')
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.traffic_pub = rospy.Publisher("traffic_data",GetTrafficLightStatus,queue_size=1)
        self.imgs=None
        rate= rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.imgs is not None:
                result = self.model(self.imgs)
                # Only traffic light
                data=result.pandas().xyxy[0][result.pandas().xyxy[0].name=="traffic light"]
                # data=result.pandas().xyxy[0][result.pandas().xyxy[0].name=="class9"]
                temp = data[data["xmax"] - data["xmin"]>data["ymax"] - data["ymin"]]
                if(len(temp)>0):
                   maxidx = ((temp["xmax"]-temp["xmin"])*(temp["ymax"]-temp["ymin"])).argmax()
                   height = temp.iloc[maxidx]["ymax"] - temp.iloc[maxidx]["ymin"]
                   width = temp.iloc[maxidx]["xmax"] - temp.iloc[maxidx]["xmin"]
                   if(width > height*1.7):
                        traffic_light = temp.iloc[maxidx]
                        traffic_status = Check_Color(self.imgs,traffic_light)
                        if(traffic_status!=0):
                            msgs = GetTrafficLightStatus()
                            msgs.trafficLightIndex = "Traffic_Light"
                            msgs.trafficLightType=1
                            msgs.trafficLightStatus=traffic_status
                            self.traffic_pub.publish(msgs)
                       
               
            else:
                print("wait data")
            rate.sleep()


    def callback(self,msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.imgs = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('lane_fitting', anonymous=True)
        od = Object_Detect()
    except rospy.ROSInterruptException:
        pass
