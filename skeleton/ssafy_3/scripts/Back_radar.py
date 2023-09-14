#!/usr/bin/env python3
#-*- coding:utf-8 -*-

from morai_msgs.msg import RadarDetections, RadarDetection
import numpy as np
import rospy
import time
import math


RIGHT_RADAR = {
    "X": -0.73,
    "Y": 0.82,
    "Z": -0.01,
    "YAW": np.radians(135), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

LEFT_RADAR = {
    "X": -0.73,
    "Y": -0.82,
    "Z": 0.00,
    "YAW": np.radians(220), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

def getTransformMat(params_sensor) :
    sensorPosition = np.array([params_sensor.get(i) for i in (["X","Y","Z"])]) 
    sensorRPY = np.array([params_sensor.get(i) for i in (["ROLL","PITCH","YAW"])])  
    cosY = math.cos(sensorRPY[2])
    sinY = math.sin(sensorRPY[2])
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
    def __init__(self): 
        rospy.Subscriber('/right_radar', RadarDetections, self.right_callback)
        rospy.Subscriber('/left_radar', RadarDetections, self.left_callback)

        self.radar_pub = rospy.Publisher('radar_data', RadarDetections, queue_size=1)
        self.point_list = None
        self.detection_list = None

        self.RightTransmMat = getTransformMat(RIGHT_RADAR)
        self.LeftTransMat = getTransformMat(LEFT_RADAR)

    def right_callback(self, msg): #RaderDetection[] RadarDetections
        print("get radar")
        now = time.time()
        ''' RaderDetection
        uint16 detection_id
        geometry_msgs/Point position
        float32 azimuth
        float32 rangerate
        float32 amplitude
        '''
        # radar_data_list = RadarDetections()
        for point in msg.detections :
            if point.position.x == 0 and point.position.y == 0 and point.position.z == 0 :
                continue
            
            tmp_data = np.array([point.position.x,point.position.y,point.position.z,1]) # 4x4
            trans_data = self.RightTransmMat.dot(tmp_data.T)
            if trans_data[1]<-6 or trans_data[0] <-10:
                continue

            print(point.detection_id,np.round(trans_data[0],4).astype(float), np.round(trans_data[1],4).astype(float), np.round(trans_data[2],4).astype(float),  point.rangerate)
            # radar_data = RadarDetection()
            # radar_data.detection_id = point.detection_id
            # radar_data.position.x = trans_data[0][0]
            # radar_data.position.y = trans_data[1][0]
            # radar_data.position.z = trans_data[2][0]

            # radar_data.azimuth = point.azimuth
            # radar_data.rangerate = point.rangerate
            # radar_data.amplitude = point.amplitude
        #     radar_data_list.detections.append(radar_data)
        # self.radar_pub.publish(radar_data_list)
        # print(radar_data_list)
        print("process time : ",time.time()-now)
    def left_callback(self, msg): #RaderDetection[] RadarDetections
        ''' RaderDetection
        uint16 detection_id
        geometry_msgs/Point position
        float32 azimuth
        float32 rangerate
        float32 amplitude
        '''
        radar_data_list = RadarDetections()
        for point in msg.detections :
            if point.position.x == 0 and point.position.y == 0 and point.position.z == 0 :
                continue
            
            tmp_data = np.array([point.position.x,point.position.y,point.position.z,1]) # 4x4

            trans_data = self.LeftTransMat.dot(tmp_data)
            if trans_data[1]<-6:
                continue
            # radar_data = RadarDetection()
            # radar_data.detection_id = point.detection_id
            # radar_data.position.x = trans_data[0][0]
            # radar_data.position.y = trans_data[1][0]
            # radar_data.position.z = trans_data[2][0]

            # radar_data.azimuth = point.azimuth
            # radar_data.rangerate = point.rangerate
            # radar_data.amplitude = point.amplitude
        #     radar_data_list.detections.append(radar_data)
        # self.radar_pub.publish(radar_data_list)
        # print(radar_data_list)


if __name__ == '__main__':
    rospy.init_node('rader', anonymous=True)
    rospy.Rate(20)
    Transformer = Radar()
    rospy.spin()
        
