#! /usr/bin/env python3
from morai_msgs.msg import GetTrafficLightStatus
import numpy as np
import rospy
import cv2


class Traffic_Light_Status:
    def __init__(self,img,data):
        self.data = data
        self.img = img

        self.lower_red = np.array([-10, 30, 50]) 
        self.upper_red = np.array([10, 255, 255])

        self.lower_green = np.array([50, 80, 80])
        self.upper_green = np.array([90, 255, 255])

        self.lower_yellow = np.array([11, 50, 50])
        self.upper_yellow = np.array([30, 200, 200])

    ## Find Lasgest Traffic Light
    def preprocessing(self):
        traffic_datas=self.data.pandas().xyxy[0][self.data.pandas().xyxy[0].name=="class9"]
        if(len(traffic_datas)==0):
            return 0
        
        traffic_datas = traffic_datas[traffic_datas["xmax"] - traffic_datas["xmin"]> (traffic_datas["ymax"] - traffic_datas["ymin"]) * 1.7]
        maxidx = ((traffic_datas["xmax"]-traffic_datas["xmin"])*(traffic_datas["ymax"]-traffic_datas["ymin"])).argmax()

        traffic_light_data = traffic_datas.iloc[maxidx]
        return traffic_light_data

    ## Find Traffic Light Status with HSV
    def check_color(self):
        traffic_light_data = self.preprocessing()
        if(traffic_light_data==0):
            return 0
        
        image = self.img[int(traffic_light_data["ymin"]):int(traffic_light_data["ymax"]),int(traffic_light_data["xmin"]):int(traffic_light_data["xmax"])]
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
    def get_traffic_light_status(self):
        msgs = GetTrafficLightStatus()
        msgs.trafficLightIndex = "Traffic_Light"
        msgs.trafficLightType=0
        msgs.trafficLightStatus=self.check_color()
        return msgs