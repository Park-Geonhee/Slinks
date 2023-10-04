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
from std_msgs.msg import Float64MultiArray
# gpsimu_parser 는 GPS, IMU 센서 데이터를 받아 차량의 상대위치를 추정하는 예제입니다.

# 노드 실행 순서 
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 송신 될 Odometry 메세지 변수 생성
# 3. 위도 경도 데이터 UTM 죄표로 변환
# 4. Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
# 5. Odometry 메세지 Publish

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
    
        # 초기화
        self.input_len = 0
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        #self.is_pinpoint=False

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        # GPS 센서에서 수신되는 위도, 경도 데이터를 UTM 좌표료 변환 하기 위한 예제이다.
        # 해당 예제는 WGS84 좌표계에서 UTM 좌표계로의 변환을 진행한다.
        # 시뮬레이터 K-City Map 의 경우 UTM 좌표계를 사용하며 실제 지도 상 위치는 UTM 좌표계의 52 Zone 에 존제한다.
        # 맵 좌표계는 m 단위를 사용한다.
        # 아래 주소의 링크를 클릭하여 Ptoj 의 사용 방법을 확인한다.
        # https://pyproj4.github.io/pyproj/stable/api/proj.html
        # " proj= , zone= , ellps =  , preserve_units = "
        # self.proj_UTM = Proj( 좌표 변환을 위한 변수 입력 )

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=True)
        self.proj_UTM2 = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=True)


        #TODO: (2) 송신 될 Odometry 메세지 변수 생성
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'
        #self.pinpoint_utm_list = Float64MultiArray()

        print(self.input_len)
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()

                #TODO: (5) Odometry 메세지 Publish

                self.odom_pub.publish(self.odom_msg)
                #if self.is_pinpoint:
                    #self.pinpoint_utm_list_pub.publish(self.pinpoint_utm_list)
                #    self.is_pinpoint = False
                rate.sleep()

    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

    #TODO: (3) 위도 경도 데이터 UTM 죄표로 변환
    def convertLL2UTM(self):
        xy_zone = self.proj_UTM(self.lon, self.lat)
        #xy_zone = self.proj_UTM(self.lon, self.lat)
        
        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

    def imu_callback(self, data):
        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
            
        self.is_imu=True
 
if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass



