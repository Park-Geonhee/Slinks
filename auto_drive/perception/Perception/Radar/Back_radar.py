#!/usr/bin/env python3
#-*- coding:utf-8 -*-

from morai_msgs.msg import RadarDetections, RadarDetection
import numpy as np
import rospy
import time
import math


RIGHT_RADAR = {
    "X": -0.73,
    "Y": -0.74,
    "Z": -0.01,
    "YAW": np.radians(220), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

LEFT_RADAR = {
    "X": -0.73,
    "Y": 0.74,
    "Z": -0.01,
    "YAW": np.radians(130), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

def getTransformMat(params_sensor) :
    RPY = np.array([params_sensor.get(i) for i in (["ROLL","PITCH","YAW"])])  
    cosR = math.cos(RPY[0])
    cosP = math.cos(RPY[1])
    cosY = math.cos(RPY[2])
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])
    
    rotRoll = np.array([[1, 0, 0], 
                       [0, cosR, -sinR], 
                       [0, sinR, cosR]])
    rotPitch = np.array([[cosP, 0, sinP],
                        [0, 1, 0], 
                        [-sinP, 0, cosP]])
    rotYaw = np.array([[cosY, -sinY, 0],
                      [sinY, cosY, 0], 
                      [0, 0, 1]])

    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))    
    sensorRotationMat = np.zeros((4, 4))
    sensorRotationMat[:3,:3] = rotMat
    sensorRotationMat[3,3] = 1
    return sensorRotationMat


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
        ''' RaderDetection
        uint16 detection_id
        geometry_msgs/Point position
        float32 azimuth
        float32 rangerate
        float32 amplitude
        '''
        radar_data_list = RadarDetections()
        # print("Right Object!!")
        for point in msg.detections :
            if point.position.x == 0 and point.position.y == 0 and point.position.z == 0 :
                continue
            
            tmp_data = np.array([point.position.x,point.position.y,point.position.z,1]) # 4x4
            trans_data = self.RightTransmMat.dot(tmp_data.T)
            if trans_data[1]<-6 :
                continue
            # print(trans_data, point.rangerate)
            # radar_data = RadarDetection()
            # radar_data.detection_id = point.detection_id
            # radar_data.position.x = trans_data[0]
            # radar_data.position.y = trans_data[1]
            # radar_data.position.z = trans_data[2]

            # radar_data.azimuth = point.azimuth
            # radar_data.rangerate = point.rangerate
            # radar_data.amplitude = point.amplitude
            # radar_data_list.detections.append(radar_data)
        
        # print(radar_data_list)
        #self.radar_pub.publish(radar_data_list)
        
    def left_callback(self, msg): #RaderDetection[] RadarDetections
        radar_data_list = RadarDetections()
        # print("Left Object!!")
        for point in msg.detections :
            if point.position.x == 0 and point.position.y == 0 and point.position.z == 0 :
                continue
            
            tmp_data = np.array([point.position.x,point.position.y,point.position.z,1]) # 4x4
            trans_data = self.RightTransmMat.dot(tmp_data.T)
            if trans_data[1]<-6 :
                continue
            # print(trans_data, point.rangerate)
            # radar_data = RadarDetection()
            # radar_data.detection_id = point.detection_id
            # radar_data.position.x = trans_data[0]
            # radar_data.position.y = trans_data[1]
            # radar_data.position.z = trans_data[2]

            # radar_data.azimuth = point.azimuth
            # radar_data.rangerate = point.rangerate
            # radar_data.amplitude = point.amplitude
            # radar_data_list.detections.append(radar_data)

        
        # print(radar_data_list)
        # self.radar_pub.publish(radar_data_list)


if __name__ == '__main__':
    rospy.init_node('rader', anonymous=True)
    time.sleep(1)
    rospy.Rate(5)
    Transformer = Radar()
    rospy.spin()
        
