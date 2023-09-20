#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import ObjectStatusList, ObjectStatus, RadarDetections

def callback(msg) :
    print("type",type(msg))
    print("msg",msg)

def obj_pc_callback(msg):
    print("\nobject_pc")
    # print("type", type(msg))
    # print("msg", msg)
    print(msg.header)

def obj_list_callback(msg):
    print("\nobject_list")
    # print("type", type(msg))
    # print("msg", msg)
    print(msg.header)

def radar_callback(msg) :
    print(msg.header)

def radar_data_callback(msg) :
    print("radar_data")
    print(msg)

def lidar_data_callback(msg) :
    print("lidar_data")
    print(msg)
rospy.init_node('test', anonymous=True)
# rospy.Subscriber('/Object_topic',ObjectStatusList,callback)
# rospy.Subscriber('/object_pc',ObjectStatusList,obj_pc_callback)
# rospy.Subscriber('/Object_topic',ObjectStatusList, obj_list_callback)
# rospy.Subscriber('/radar',RadarDetections,radar_callback)
rospy.Subscriber('/radar_detection',ObjectStatusList,radar_data_callback)
rospy.Subscriber('/lidar_detection',ObjectStatusList,lidar_data_callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pass
