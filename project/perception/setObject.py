#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
import time
from morai_msgs.msg import ObjectStatusList, ObjectStatus, RadarDetections, RadarDetection
from geometry_msgs.msg import PoseArray,Pose, Point32
from sensor_msgs.msg import PointCloud2, PointCloud
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler
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
    "X": 1., # meter
    "Y": 0.,
    "Z": 1.3,
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
        rospy.Subscriber('/clusters', PoseArray, self.cluster_callback)
        # rospy.Subscriber('/radar_data',RadarDetections, self.radar_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        # self.radar_detect_pub = rospy.Publisher('radar_detection',ObjectStatusList, queue_size=1)
        self.lidar_detect_pub = rospy.Publisher('lidar_detection',ObjectStatusList, queue_size=1)
        # self.radar_pc_pub = rospy.Publisher('radar_pc',PointCloud, queue_size=1)
        self.lidar_pc_pub = rospy.Publisher('lidar_pc',PointCloud, queue_size=1)
        self.lidar_status = False
        self.radar_status = False
        self.lidar_data = None
        self.radar_data = None
        self.is_odom = False
        rate = rospy.Rate(10)
        cnt =0
        while not rospy.is_shutdown():
            if self.is_odom == False :
                continue
            if self.lidar_status == False and self.radar_status == False :
                continue

            if self.lidar_status == True :
                lidar_list = ObjectStatusList()
                lidar_pc = PointCloud()
                lidar_list.num_of_obstacle = len(self.lidar_data.poses)
                lidar_pc.header.frame_id='map'
                for i in self.lidar_data.poses :
                    tmp_detection = self.get_global_pose(i)
                    tmp_point = self.get_global_lidar(tmp_detection,"point")
                    tmp_obstacle = self.get_global_lidar(tmp_detection,"objectStatus")
                    lidar_pc.points.append(tmp_point)
                    lidar_list.obstacle_list.append(tmp_obstacle)

                self.lidar_detect_pub.publish(lidar_list)
                self.lidar_pc_pub.publish(lidar_pc)
                print("========lidar_pc")
                print(lidar_pc)

            if self.radar_status == True :
                radar_list = ObjectStatusList()
                radar_pc = PointCloud()
                radar_list.num_of_obstacle = len(self.radar_data.detections)
                radar_pc.header.frame_id='map'

                for i in self.radar_data.detections :

                    tmp_detection = self.get_global_detection(i) # 
                    tmp_detection.rangerate = i.rangerate
                    tmp_point = self.get_global_radar(tmp_detection,"point")   
                    tmp_obstacle = self.get_global_radar(tmp_detection,"objectStatus")
                    radar_pc.points.append(tmp_point)
                    radar_list.obstacle_list.append(tmp_obstacle)
  
                self.radar_detect_pub.publish(radar_list)
                self.radar_pc_pub.publish(radar_pc)
                print("========radar_pc")
                print(radar_pc)
                # print("========radar_list")
                # print(radar_list)

    def cluster_callback(self, msg) :
        self.lidar_data = msg # PoseArray
        self.lidar_status = True

    def radar_callback(self, msg) :
        self.radar_data = msg # RadarDetections
        self.radar_status = True

    def odom_callback(self, msg):
        if self.is_odom == False:
            print(msg.pose.pose)
        self.is_odom = True
        odom_quaternion=(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.vehicle_pos_x = msg.pose.pose.position.x
        self.vehicle_pos_y = msg.pose.pose.position.y
        self.vehicle_pos_z = msg.pose.pose.position.z
        # print(msg.pose.pose)

    def set_Point32(self, detection):
        data = Point32() # float32
        data.x = detection.position.x
        data.y = detection.position.y
        data.z = detection.position.z
        return data
    
    def set_ObjectStatus(self, detection):
        data = ObjectStatus()
        data.type = 2
        data.position.x = detection.position.x
        data.position.y = detection.position.y
        data.position.z = detection.position.z
        return data
    def get_global_pose(self, pose):
        data = Pose()
        trans_matrix = np.array([
            [np.cos(self.vehicle_yaw), -np.sin(self.vehicle_yaw), 0, self.vehicle_pos_x],
            [np.sin(self.vehicle_yaw), np.cos(self.vehicle_yaw), 0, self.vehicle_pos_y],
            [0, 0, 1, self.vehicle_pos_z],
            [0, 0, 0, 1]
            ])
        local_result = np.array([[pose.position.x],[pose.position.y],[pose.position.z],[1]])
        global_result = trans_matrix.dot(local_result)
        data.position.x = global_result[0][0]
        data.position.y = global_result[1][0]
        data.position.z = global_result[2][0]
        return data
    def get_global_detection(self, detection):
        data = RadarDetection()
        trans_matrix = np.array([
            [np.cos(self.vehicle_yaw), -np.sin(self.vehicle_yaw), 0, self.vehicle_pos_x],
            [np.sin(self.vehicle_yaw), np.cos(self.vehicle_yaw), 0, self.vehicle_pos_y],
            [0, 0, 1, self.vehicle_pos_z],
            [0, 0, 0, 1]
            ])

        local_result = np.array([[detection.position.x],[detection.position.y],[detection.position.z],[1]])
        global_result = trans_matrix.dot(local_result)
        data.position.x = global_result[0][0]
        data.position.y = global_result[1][0]
        data.position.z = global_result[2][0]
        data.rangerate = detection.rangerate
        return data
    
    def get_global_lidar(self, detection, type) : 
        result = None
        if type == "point" : # Point32 pose
            result = self.set_Point32(detection)
        elif type == "objectStatus" : # ObjectStatus
            result = self.set_ObjectStatus(detection)
            result.name = "lidar_data"
        return result
    
    def get_global_radar(self, detection, t) : 
        result = None
        if t == "point" : # Point32 pose
            result = self.set_Point32(detection)
        elif t == "objectStatus" : # ObjectStatus
            result = self.set_ObjectStatus(detection)
            result.velocity.x = detection.rangerate
            result.name = "radar_data"
        return result
    
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