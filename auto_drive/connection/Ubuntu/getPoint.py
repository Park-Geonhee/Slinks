#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi



class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        #rospy.Subscriber("/imu", Imu, self.imu_callback)
        # 웹에서 출발지, 경유지, 도착지 좌표(gps)를 받는 subscriber 필요
        rospy.Subscriber("/point", Float64MultiArray, self.getPoint_list_callback)

        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.pinpoint_utm_list_pub = rospy.Publisher('/pinpoint_utm_list',Float64MultiArray, queue_size=1)
    
        # 초기화
        self.input_len = 0
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        self.is_pinpoint=False


        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=True)
       


    
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'
        self.pinpoint_utm_list = Float64MultiArray()


        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_pinpoint==True:
                self.convertLL2UTM()
                self.pinpoint_utm_list_pub.publish(self.pinpoint_utm_list)
                self.is_pinpoint = False
            rate.sleep()


    def navsat_callback(self, gps_msg):

        #self.lat = gps_msg.latitude
        #self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True



    #TODO: (3) 위도 경도 데이터 UTM 죄표로 변환
    def convertLL2UTM(self):

        if self.is_pinpoint == True:
            self.pinpoint_utm_list = Float64MultiArray()
            for i in range(self.input_len//2):
                pp_x = self.pinpoint_list[i*2]
                pp_y = self.pinpoint_list[i*2+1]
                pp_xy = self.proj_UTM(pp_y, pp_x)
                
                if pp_x == 0 and pp_y == 0:
                    pp_utm_x = 0.0
                    pp_utm_y = 0.0
                else:
                    pp_utm_x = pp_xy[0] - self.e_o
                    pp_utm_y = pp_xy[1] - self.n_o
                self.pinpoint_utm_list.data.append(pp_utm_y)
                self.pinpoint_utm_list.data.append(pp_utm_x)
            

    def getPoint_list_callback(self, msg):
        self.pinpoint_list = msg.data
        self.input_len = len(self.pinpoint_list)
        self.is_pinpoint = True
        print(self.input_len)

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass

