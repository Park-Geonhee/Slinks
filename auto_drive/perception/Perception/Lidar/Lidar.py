#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN

# lidar_velodyne_cluster 는 LiDAR의 Point들을 물체 단위로 구분하는 Clustering 예제입니다.
# PointCloud Data를 입력받아 DBSCAN Algorithm을 활용하여 Clustering을 수행합니다.
# 교육생분들은 DBSCAN의 Parameter를 조절하여 적절한 Clustering 결과를 얻어내야 합니다.

# 노드 실행 순서
# 1. DBSCAN Parameter 입력
# 2. 각 Cluster를 대표하는 위치 값 계산
# 3. PointCloud Data로부터 Distance, Angle 값 계산
parameters_lidar = {
    "X": 1., # meter
    "Y": 0.,
    "Z": 1.3,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}
class Lidar:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/lidar3D", PointCloud2, self.callback)
        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10) 
        self.pc_np = None

        #TODO: (1) DBSCAN Parameter 입력
        
        # DBSCAN의 Parameter를 결정하는 영역입니다.
        # sklearn.cluster의 DBSCAN에 대해 조사하여 적절한 Parameter를 입력하기 바랍니다.
        # epsilon, min_samples, metric='euclidean', metric_params=None, algorithm='auto', leaf_size=30, p=None, n_jobs=None
        self.dbscan = DBSCAN(eps=0.5, min_samples=5) 
        cosY = math.cos(parameters_lidar["YAW"])
        sinY = math.sin(parameters_lidar["YAW"])
        posX = parameters_lidar['X']
        posY = parameters_lidar['Y']
        posZ = parameters_lidar['Z']

        self.trans_matrix = np.array([
            [cosY, -sinY, 0, posX],
            [sinY, cosY, 0, posY],
            [0, 0, 1, posZ],
            [0, 0, 0, 1]
        ])
    def callback(self, msg):    
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            cluster_msg = PoseArray() # header, poses[]

        else:
            pc_xyz = self.pc_np[:, :3] # all rows, 0~1 col -> each point's coordinate
            db = self.dbscan.fit_predict(pc_xyz[:, :2]) # each point's cluster group 
           
            n_cluster = np.max(db) + 1 # cluster group cnt
           
            cluster_msg = PoseArray() # header, poses[]
            cluster_list = []

            print('start')
            for cluster in range(n_cluster):
                #TODO: (2) 각 Cluster를 대표하는 위치 값 계산                
                cluster_points = pc_xyz[db==cluster]
                
                cluster_center_x = np.mean(cluster_points[:,0])
                cluster_center_y = np.mean(cluster_points[:,1])
                cluster_center_z = np.mean(cluster_points[:,2])

                # Input : cluster
                # Output : cluster position x,y   
                local_result = np.array([[cluster_center_x],[cluster_center_y],[cluster_center_z],[1]])
                local_result = self.trans_matrix.dot(local_result) # local coordi to clusters msg
               
                tmp_pose=Pose() # Point position (x,y,z), Quaternion orientation(x,y,z,w)
                tmp_pose.position.x = local_result[0][0]
                tmp_pose.position.y = local_result[1][0]
                tmp_pose.position.z = local_result[2][0]
                print(tmp_pose)

                cluster_msg.poses.append(tmp_pose)  
        self.cluster_pub.publish(cluster_msg)  
        
    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True): 
            # point : 'x', 'y', 'z', "intensity"
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            
            # LiDAR의 PointCloud Data로부터 Distance와 Angle 값을 계산하는 영역입니다.
            # 각 Point의 XYZ 값을 활용하여 Distance와 Yaw Angle을 계산합니다.
            # Input : point (X, Y, Z, Intensity)     
            xy_dist = (point[0]**2+point[1]**2)**0.5
            dist = (xy_dist**2+point[2]**2)**0.5
            if point[0]==0 : continue
            angle = math.atan(point[1]/point[0])
            
            
            if point[0] > 0 and 2 > point[2] > -1.25 and dist < 50: # 
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np



if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)
    scan_cluster = Lidar()
    rospy.spin() 
