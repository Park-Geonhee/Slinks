#! /usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import torch
import numpy as np
import os

from cv_bridge import CvBridgeError
from sensor_msgs.msg import CompressedImage

lower_red = np.array([-10, 30, 50]) 
upper_red = np.array([10, 255, 255])

lower_green = np.array([50, 80, 80])
upper_green = np.array([90, 255, 255])

lower_yellow = np.array([11, 50, 50])
upper_yellow = np.array([30, 200, 200])

PATH = "/home/lsh/catkin_ws/src/test_pkg/src/yolov5"
os.chdir(PATH)

def Check_Color(img, traffic_light):
    try:
        img = img[int(traffic_light["ymin"]):int(traffic_light["ymax"]),int(traffic_light["xmin"]):int(traffic_light["xmax"])]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    except:
        return ""
    
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
    
    if(flag==1):
        return "red"
    elif(flag==2):
        return "green"
    elif(flag==3):
        return "left"
    elif(flag==4 or flag==5):
        return "yellow"
    else:
        return ""
    
class Object_Detect:
    def __init__(self):
        self.model = torch.hub.load('', 'custom', 'yolov5s.onnx',source='local')
        # self.model = torch.hub.load('', 'yolov5s', source='local')
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.imgs=None
        rate= rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.imgs is not None:
                result = self.model(self.imgs)
                # Only traffic light
                data=result.pandas().xyxy[0][result.pandas().xyxy[0].name=="traffic light"]

                temp = data[data["xmax"] - data["xmin"]>data["ymax"] - data["ymin"]]
                if(len(temp)>0):
                   maxidx = ((temp["xmax"]-temp["xmin"])*(temp["ymax"]-temp["ymin"])).argmax()
                   traffic_light = temp.iloc[maxidx]
                   traffic_status = Check_Color(self.imgs,traffic_light)
                   if(traffic_status!=""):
                       print(traffic_status)
                       
                #    cv2.rectangle(self.imgs,(int(traffic_light["xmin"]),int(traffic_light["ymin"])),(int(traffic_light["xmax"]),int(traffic_light["ymax"])),(255,0,0),1)
                #    cv2.putText(self.imgs,text=traffic_status+" traffic light",org=(int(traffic_light["xmin"])-10,int(traffic_light["ymin"])),color=(255,0,0),fontFace=0,fontScale=0.5)

                data=result.pandas().xyxy[0][result.pandas().xyxy[0].name=="person"]
                if(len(data)>0):
                   for i in range(len(data)):
                       d = data.iloc[i]
                       x = int((d["xmax"]+d["xmin"] )/2)
                       y = int((d["ymax"]+d["ymin"] )/2)
                       print(f"Find Person Coord : {x} , {y} ")
                    #    cv2.rectangle(self.imgs,(int(d["xmin"]),int(d["ymin"])),(int(d["xmax"]),int(d["ymax"])),(0,0,255),1)
                    #    cv2.putText(self.imgs,text="pedes",org=(int((d["xmin"]))-10,(int(d["ymin"]))),color=(0,0,255),fontFace=0,fontScale=0.5)
                # cv2.imshow("od",self.imgs)
                # cv2.waitKey(1)
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

