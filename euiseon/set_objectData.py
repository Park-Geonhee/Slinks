#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
import time
from morai_msgs.msg import ObjectStatusList, ObjectStatus, RadarDetections
from geometry_msgs.msg import PoseArray,Pose, Point32
from sensor_msgs.msg import PointCloud2, PointCloud
'''
using camera(2D object detect), lidar(object position), radar(velocity) data,
identify objects around ego vehicle

1. camera object detect (2D image coordi)
2. lidar object detect (3D local coordi)
3. radar object detect (3D local coordi)
4. transform the object's coordi to camera image 
5. connect lidar's 2D data with detected object in image
6. ros publish object info in ObjectStatusList msg
'''
parameters_cam ={
    "WIDTH": 640, #image width
    "HEIGHT": 480, #image height
    "FOV": 90, # Field of views
    "X": 3.67, # meter
    "Y": -0.01,
    "Z": 0.51,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}
parameters_lidar = {
    "X": 1.58, # meter
    "Y": -0.01,
    "Z": 1.07,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}
parameters_radar = {
    "X": 3.68, # meter
    "Y": -0.14,
    "Z": 0.51,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

class ObjectDataSet :
    def __init__(self, params_cam, params_lidar, params_radar ) :
        self.cluster_sub = rospy.Subscriber('/clusters', PoseArray, self.cluster_callback)
        self.radar_sub = rospy.Subscriber('/radar_data',RadarDetections, self.radar_callback)
        self.object_pc_pub = rospy.Publisher('object_pc', PointCloud, queue_size = 1)
        self.object_list_pub = rospy.Publisher('object_list',ObjectStatusList, queue_size=1)
        self.lidar_status = False
        self.radar_status = False
        self.lidar_data = None
        self.radar_data = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.lidar_status == False and self.radar_status == False :
                pass
            obj_pc = PointCloud() # for connecting pc with camera image after transform to camera image  
            obj_pc.header.frame_id='map'

            obj_list = ObjectStatusList()
            obj_list.num_of_obstacle = 0
            
            if self.lidar_status == True :
                obj_list.num_of_obstacle = len(self.lidar_data.poses)
                for i in self.lidar_data.poses :
                    tmp_point = Point32() # float32
                    tmp_point.x = i.position.x
                    tmp_point.y = i.position.y
                    tmp_point.z = i.position.z
                    obj_pc.points.append(tmp_point)

                    tmp_obstacle = ObjectStatus()
                    tmp_obstacle.type = 2
                    tmp_obstacle.position.x = i.position.x
                    tmp_obstacle.position.y = i.position.y
                    tmp_obstacle.position.z = i.position.z
                    tmp_obstacle.name = "lidar_data"
                    obj_list.obstacle_list.append(tmp_obstacle)

            if self.radar_status == True :
                obj_list.num_of_obstacle = obj_list.num_of_obstacle + len(self.radar_data.detections)
                for i in self.radar_data.detections :
                    tmp_point = Point32() # float32
                    tmp_point.x = i.position.x
                    tmp_point.y = i.position.y
                    tmp_point.z = i.position.z
                    obj_pc.points.append(tmp_point)

                    tmp_obstacle = ObjectStatus()
                    tmp_obstacle.type = 2
                    tmp_obstacle.position.x = i.position.x
                    tmp_obstacle.position.y = i.position.y
                    tmp_obstacle.position.z = i.position.z
                    tmp_obstacle.velocity.x = i.rangerate
                    tmp_obstacle.name = "radar_data"
                    obj_list.obstacle_list.append(tmp_obstacle)
            
            self.object_list_pub.publish(obj_list)
            print("obj_list", obj_list)
            self.object_pc_pub.publish(obj_pc)

    def cluster_callback(self, msg) :
        self.lidar_data = msg # PoseArray
        self.lidar_status = True

    def radar_callback(self, msg) :
        self.radar_data = msg # RadarDetections
        self.radar_status = True

if __name__ == '__main__':
    rospy.init_node('object_data', anonymous=True)
    object_data_setting = ObjectDataSet(parameters_cam, parameters_lidar, parameters_radar)
    rospy.spin()

''' msg type
- PoseArray
Header header
Pose[] poses

- Pose
Point position (x,y,z)
Quaternion orientation (x,y,z,w)

-RadarDetections
Header header
RaderDetection[] detections

-RadarDetection
uint16 detection_id
geometry_msgs/Point position
float32 azimuth
float32 rangerate
float32 amplitude

-Point
float64 x
float64 y
float64 z

-ObjectStatus
int32 unique_id
int32 type
string name
float64 heading

geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 size
geometry_msgs/Vector3 position

-Vector3
float64 x
float64 y
float64 z

-Header
uint32 seq # continuesly increasing count
time stamp # secs
string frame_id # 

-PointCloud

'''