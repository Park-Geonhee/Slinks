#!/usr/bin/env python3
#-*- coding:utf-8 -*-

from morai_msgs.msg import GetTrafficLightStatus
import numpy as np
import cv2

class TrafficLight:
    def __init__(self):

        self.lower_red = np.array([-10, 30, 50]) 
        self.upper_red = np.array([10, 255, 255])

        self.lower_green = np.array([50, 80, 80])
        self.upper_green = np.array([90, 255, 255])

        self.lower_yellow = np.array([11, 50, 50])
        self.upper_yellow = np.array([30, 200, 200])

    ## Find Lasgest Traffic Light
    def preprocessing(self, data):
        traffic_datas=data[data.name=="traffic light"]
        if(len(traffic_datas)==0):
            return None
        
        try:
            traffic_datas = traffic_datas[traffic_datas["xmax"] - traffic_datas["xmin"]> (traffic_datas["ymax"] - traffic_datas["ymin"]) * 1.7]
            maxidx = ((traffic_datas["xmax"]-traffic_datas["xmin"])*(traffic_datas["ymax"]-traffic_datas["ymin"])).argmax()

            traffic_light_data = traffic_datas.iloc[maxidx]
            return traffic_light_data
        except:
            return None

    ## Find Traffic Light Status with HSV
    def check_color(self, img, data):
        traffic_light_data = self.preprocessing(data)
        if(traffic_light_data is None):
            return 0
        
        image = img[int(traffic_light_data["ymin"])+3:int(traffic_light_data["ymax"])-3,int(traffic_light_data["xmin"])+5:int(traffic_light_data["xmax"])-5]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
        red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        red_areas = cv2.bitwise_and(image, image, mask=red_mask)

        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        green_areas = cv2.bitwise_and(image, image, mask=green_mask)

        yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        yellow_areas = cv2.bitwise_and(image, image, mask=yellow_mask)

        flag=0

        if np.any(green_areas):
            flag+=1
        if np.any(red_areas):
            flag+=2
        if np.any(yellow_areas):
            flag+=4

        return flag

    ## Return Traffic Light Status
    def get_traffic_light_status(self, img, data):
        msgs = GetTrafficLightStatus()
        msgs.trafficLightIndex = "Traffic_Light"
        msgs.trafficLightType=0
        msgs.trafficLightStatus=self.check_color(img, data)
        return msgs