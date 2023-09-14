#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math


class getFunc :
    def __init__(self) :
        pass

    def getRotMat(self, params_sensor): #3x3 
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
        return rotMat
    
    def getSensorToVehicleMat(self, params_sensor): # 4x4
        sensorRotationMat = self.getRotMat(params_sensor)

        sensorPosition = np.array([params_sensor.get(i) for i in (["X","Y","Z"])])
        sensorTranslationMat = np.array([sensorPosition])
        Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat,sensorTranslationMat.T),axis = 1)
        Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0,0,0,1],axis = 0)
        
        return Tr_sensor_to_vehicle