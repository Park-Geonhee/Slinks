#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from morai_msgs.msg import RadarDetections
from numpy.linalg import inv
from geometry_msgs.msg import Point

BACK_RADAR1 = {
    "X": -0.72,
    "Y": -0.74,
    "Z": -0.07,
    "YAW": np.radians(220), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

BACK_RADAR2 = {
    "X": 3.25,
    "Y": -0.02,
    "Z": 0.03,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

## rotation mat
def getRotMat(RPY):        
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
    return rotMat

## TransForm MAT
def getSensorToVehicleMat(sensorRPY, sensorPosition):
    # Roatation MaT
    sensorRotationMat = getRotMat(sensorRPY)
    # Translate
    sensorTranslationMat = np.array([sensorPosition]) ## Coord
    
    Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat,sensorTranslationMat.T),axis = 1)
    Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0,0,0,1],axis = 0)
    return Tr_sensor_to_vehicle


def getTransformMat(params_RADAR):
    radarPosition = np.array([params_RADAR.get(i) for i in (["X","Y","Z"])])    
    radarRPY = np.array([params_RADAR.get(i) for i in (["ROLL","PITCH","YAW"])])
    
    print("radarRPY:",radarRPY) #radarRPY [Roll, pitch, YAW]
    print("radarPosition:",radarPosition)
    Tr_radar_to_vehicle = getSensorToVehicleMat(radarRPY,radarPosition)
    return Tr_radar_to_vehicle

class LiDARToCameraTransform:
    def __init__(self, params_RADAR): 
        self.TransformMat = getTransformMat(BACK_RADAR1)
        print(self.TransformMat)
        rospy.Subscriber('/back_radar1',RadarDetections, self.callback)

    def callback(self,msg):
        for idx,d in enumerate(msg.detections):
            d_p = np.array([d.position.x,d.position.y,d.position.z,1])
            if d_p[0]==0 and d_p[1]==0 and d_p[2]==0:
                continue
            print(self.TransformMat.dot(d_p)[:3],d.rangerate)

    # def transformLiDARToCamera(self, pc_lidar):
    #     pc_wrt_cam = self.TransformMat.dot(pc_lidar)
    #     pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
    #     return pc_wrt_cam


if __name__ == '__main__':
    
    rospy.init_node('ex_calib', anonymous=True)
    print("init node")
    
    Transformer = LiDARToCameraTransform(BACK_RADAR2)
    time.sleep(1)
    rate = rospy.Rate(20)
    rospy.spin()
    #     #xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]>10),axis=1)
    #     #xyz_p = np.delete(xyz_p,np.where(xyz_p[2,:]<-1.2),axis=1) #Ground Filter
    #     #print(xyz_p[0])

    #     xyz_c = Transformer.transformLiDARToCamera(xyz_p)
    #     # print(xyz_c)
    #     #print(np.size(xyz_c[0]))

    #     xy_i = Transformer.transformCameraToImage(xyz_c)
    #     # #print(np.size(xy_i[0]))
    #     # xy_i = np.delete(xy_i, np.where(xy_i[0,:]>400),axis=1)
    #     # xy_i = np.delete(xy_i, np.where(xy_i[1,:]>300),axis=1)
    #     # print(xy_i)
    #     #TODO: (6) PointCloud가 Image에 투영된 Processed Image 시각화
    #     xy_i = xy_i.astype(np.int32)
    #     projectionImage = draw_pts_img(Transformer.img, xy_i[0,:], xy_i[1,:])   
    
    #     cv2.imshow("LidartoCameraProjection", projectionImage)
    #     cv2.waitKey(1)

