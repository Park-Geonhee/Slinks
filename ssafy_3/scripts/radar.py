#!/usr/bin/env python3
#-*- coding:utf-8 -*-

'''
RaderDetections.msg

Header header
RadarDetectionp[] RadarDetections
'''
'''
cluster
'''
import rospy
import cv2
import numpy as np
import math
import time
from morai_msgs.msg import RadarDetections, RadarDetection
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv

parameters_radar = {
    "X": 3.68, # meter
    "Y": -0.14,
    "Z": 0.51,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

def getTransformMat(params_sensor) :
    sensorPosition = np.array([params_sensor.get(i) for i in (["X","Y","Z"])]) 
    sensorRPY = np.array([params_sensor.get(i) for i in (["ROLL","PITCH","YAW"])])  
    cosY = math.cos(sensorRPY[0])
    sinY = math.sin(sensorRPY[0])
    posX = sensorPosition[0]
    posY = sensorPosition[1]
    posZ = sensorPosition[2]
    trans_matrix = np.array([
            [cosY, -sinY, 0, posX],
            [sinY, cosY, 0, posY],
            [0, 0, 1, posZ],
            [0, 0, 0, 1]
        ])
    return trans_matrix

class Radar :
    def __init__(self, params_radar): 
        self.radar_sub = rospy.Subscriber('/radar', RadarDetections, self.scan_callback)
        self.radar_pub = rospy.Publisher('radar_data', RadarDetections, queue_size=1)
        self.point_list = None
        self.detection_list = None
        # self.TransformMat = getTransformMat(params_radar) 3x3
        self.TransformMat = getTransformMat(params_radar)

    def scan_callback(self, msg): #RaderDetection[] RadarDetections
        ''' RaderDetection
        uint16 detection_id
        geometry_msgs/Point position
        float32 azimuth
        float32 rangerate
        float32 amplitude
        '''
        # print(msg)
        radar_data_list = RadarDetections()
        for point in msg.detections :
            if point.position.x == 0 and point.position.y == 0 and point.position.z == 0 :
                continue
            
            # tmp_data = np.array([[point.position.x],[point.position.y],[1]]) #3x3
            tmp_data = np.array([[point.position.x],[point.position.y],[point.position.z],[1]]) # 4x4
            print("tmp",tmp_data)
            trans_data = self.TransformMat.dot(tmp_data)
            radar_data = RadarDetection()
            radar_data.detection_id = point.detection_id
            radar_data.position.x = trans_data[0][0]
            radar_data.position.y = trans_data[1][0]
            radar_data.position.z = trans_data[2][0]

            radar_data.azimuth = point.azimuth
            radar_data.rangerate = point.rangerate
            radar_data.amplitude = point.amplitude
            radar_data_list.detections.append(radar_data)
        self.radar_pub.publish(radar_data_list)
        print(radar_data_list)

if __name__ == '__main__':
    rospy.init_node('rader', anonymous=True)
    Transformer = Radar(parameters_radar)
    rospy.spin()
        
