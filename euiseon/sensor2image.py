#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2, CompressedImage, PointCloud
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv

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
    "X": 3.68, # meter
    "Y": -0.14,
    "Z": 0.51,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}

def getRotMat(RPY):        
    #TODO: (2.1.1) Rotation Matrix 계산 함수 구현
    print(RPY)
    # Rotation Matrix를 계산하는 영역입니다.
    # 각 회전에 대한 Rotation Matrix를 계산하면 됩니다.
    # Input
        # RPY : sensor orientation w.r.t vehicle. (Roll, Pitch, Yaw)
    # Output
        # rotMat : 3x3 Rotation Matrix of sensor w.r.t vehicle.
    # Tip : math, numpy
    # reference : https://msl.cs.uiuc.edu/planning/node102.html   

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

def getSensorToVehicleMat(sensorRPY, sensorPosition): # 4x4
    # Sensor To Vehicle Transformation Matrix를 계산하는 영역입니다.
    # sensorRotationMat = np.zeros((4, 4))
    # sensorRotationMat[:3, :3] = getRotMat(sensorRPY)
    # sensorRotationMat[3, 3] = 1
    # sensorTranslationMat = np.zeros((4, 4))
    # insert_col = [sensorPosition[0], sensorPosition[1], sensorPosition[2]]
    # sensorTranslationMat[:3,3] = insert_col
    # for i in range(4):
    #     sensorTranslationMat[i, i] = 1
    # # print("Rot",sensorRotationMat)
    # # print("Tr",sensorTranslationMat)
    # Tr_sensor_to_vehicle = sensorTranslationMat.dot(sensorRotationMat)

    sensorRotationMat = getRotMat(sensorRPY)
    sensorTranslationMat = np.array([sensorPosition])
    Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat,sensorTranslationMat.T),axis = 1)
    Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0,0,0,1],axis = 0)
    
    # print("sensorToveh",Tr_sensor_to_vehicle)
    return Tr_sensor_to_vehicle

def getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition):
    #TODO: (2.2) LiDAR to Camera Transformation Matrix 계산
    # LiDAR to Camera Transformation Matrix를 계산하는 영역입니다.
    # 아래 순서대로 계산하면 됩니다.
        # 1. LiDAR to Vehicle Transformation Matrix 계산        
        # 2. Camera to Vehicle Transformation Matrix 계산
        # 3. Vehicle to Camera Transformation Matrix 계산
        # 3. LiDAR to Camera Transformation Matrix 계산
    # Input
        # camRPY : Orientation
        # camPosition
        # lidarRPY
        # lidarPosition
    # Output
        # Tr_lidar_to_cam
    # Tip : getSensorToVehicleMat, inv 사용 필요
    # inv : matrix^-1

    # lidar = np.array([1.58,-0.01,1.07,1])
    Tr_lidar_to_vehicle = getSensorToVehicleMat(lidarRPY, lidarPosition)
    print("type",type(Tr_lidar_to_vehicle[0][0]))
    print("l to v", Tr_lidar_to_vehicle)
    # print("lidar X Tr_lidar_to_vehicle",lidar.dot(Tr_lidar_to_vehicle))
    Tr_cam_to_vehicle = getSensorToVehicleMat(camRPY, camPosition)
    Tr_vehicle_to_cam = inv(Tr_cam_to_vehicle)
    print("v to c", Tr_vehicle_to_cam)
    Tr_lidar_to_cam = Tr_vehicle_to_cam.dot(Tr_lidar_to_vehicle)
    print("l to c", Tr_lidar_to_cam)
    print("test", Tr_lidar_to_cam.dot(np.array([14.72,-1.4,0,1]).T))
    exit(1)
    # Tr_lidar_to_cam = np.cross(Tr_lidar_to_vehicle,Tr_vehicle_to_cam)
    
    return Tr_lidar_to_cam


def getTransformMat(params_cam, params_lidar):
    #With Respect to Vehicle ISO Coordinate    
    lv=-0.25
    cv=0.1085
    lidarPositionOffset = np.array([0, 0, -0.25 ]) # VLP16 사용해야 함
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset  x,y,z

    camPosition = np.array([params_cam.get(i) for i in (["X","Y","Z"])]) + camPositionOffset    
    camRPY = np.array([params_cam.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*math.pi/180,0,-90*math.pi/180])
    lidarPosition = np.array([params_lidar.get(i) for i in (["X","Y","Z"])]) 
    lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([0,0,0])   
    Tr_lidar_to_cam = getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)

    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    #TODO: (3) Intrinsic : Camera Matrix (Camera to Image Plane) 계산

    # Camera의 Intrinsic parameter로 이루어진 Camera Matrix를 계산하는 영역입니다.
    # Camera의 width, height, fov 값을 활용하여 focal length, principal point를 계산한 뒤,
    # 이를 조합하여 Camera Matrix를 생성합니다.
    # Camera Model은 Lens 왜곡이 없는 Pinhole Model임을 참고하시기 바랍니다.
    # Input
        # params_cam : camera parameters
    # Output
        # CameraMat : 3x3 Intrinsic Matrix(a.k.a. Camera Matrix)    

    camera_width = params_cam['WIDTH']
    camera_height = params_cam['HEIGHT']
    fov = params_cam['FOV']

    # FOV를 라디안으로 변환
    fov_rad = np.radians(fov)
    fov_x = np.radians(fov)
    fov_y = np.radians(fov * (float(camera_height) / camera_width)) # 480/640 *90 -> (3/4)*90 
    # 초점거리(focal length)를 계산합니다.
    focal_length = (camera_width / 2) / np.tan(fov_x / 2)

    # 이미지 중심을 기준으로한 principal point를 계산
    principal_x = camera_width/2
    principal_y = camera_height/2


    CameraMat = np.array([[focal_length, 0, principal_x],
                          [0, focal_length, principal_y],
                          [0, 0, 1]])
    
    return CameraMat
    
def getVehicleToCameraMat(params_sensor):
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset  x,y,z
    sensorPosition = np.array([params_sensor.get(i) for i in (["X","Y","Z"])]) + camPositionOffset  
    sensorRPY = np.array([params_sensor.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*math.pi/180,0,-90*math.pi/180])
    mat = getSensorToVehicleMat(sensorRPY, sensorPosition)
    mat = inv(mat)

    return mat
class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar): 
        self.scan_sub = rospy.Subscriber("/lidar3D", PointCloud2, self.scan_callback)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        # self.obj_sub = rospy.Subscriber("/object_pc", PointCloud, self.obj_pc_callback)
        self.lidar_pc_sub = rospy.Subscriber("/lidar_pc", PointCloud, self.lidar_pc_callback)
        self.radar_pc_sub = rospy.Subscriber("/radar_pc", PointCloud, self.radar_pc_callback)
        self.pc_np = None
        self.lidar_pc = None
        self.radar_pc = None
        self.img = None
        self.img_status = False
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.vehicleToCameraMat = getVehicleToCameraMat(params_cam)
        self.CameraMat = getCameraMat(params_cam)
        
        print("init")

    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img_status = True

    def scan_callback(self, msg):
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2], 1))
        self.pc_np = np.array(point_list, np.float32)

    def lidar_pc_callback(self, msg):
        point_list = []
        for point in msg.points:
            point_list.append((point.x, point.y, point.z, 1))
        self.lidar_pc = np.array(point_list, np.float32)

    def radar_pc_callback(self, msg):
        point_list = []
        for point in msg.points:
            point_list.append((point.x, point.y, point.z, 1))
        self.radar_pc = np.array(point_list, np.float32)
    #TODO : (5.1) LiDAR Pointcloud to Camera Frame
    # Input
        # pc_lidar : pointcloud data w.r.t. lidar frame
    # Output
        # pc_wrt_cam : pointcloud data w.r.t. camera frame
    def transformLiDARToCamera(self, pc_lidar):
        pc_wrt_cam = self.TransformMat.dot(pc_lidar)
        pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
        return pc_wrt_cam

    #TODO : (5.2) Camera Frame PointCloud to Image Plane with Filtering
    # Input
        # pc_camera : pointcloud data w.r.t. camera frame
    # Output
        # pc_proj_to_img : projection lidar data to image plane
    # Tip : for clear data use filtering
    def transformCameraToImage(self, pc_camera):
        pc_proj_to_img = self.CameraMat.dot(pc_camera)

        # print("img after transform",pc_proj_to_img) # 3
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[2,:]<0),axis=1)
        pc_proj_to_img /= (pc_proj_to_img[2,:])
        # print(len(pc_proj_to_img[0])) #5367
        # print("img after transform2",pc_proj_to_img) # 3
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[0,:]>self.width),axis=1)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[1,:]>self.height),axis=1)
        # print("img after transform",pc_proj_to_img) # 3
        return pc_proj_to_img

    
def draw_pts_img(img, xi, yi, color): # color : (a,b,c) RGB 
    point_np = img

    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, color,-1)
    return point_np

if __name__ == '__main__':
    rospy.init_node('ex_calib', anonymous=True)
    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    time.sleep(1)
    rate = rospy.Rate(20)
    cnt = 0
    while not rospy.is_shutdown():

        # xyz_p = Transformer.pc_np[:, 0:3] # for all rows, 0~2 col datas -> create new array
        # xyz_p = np.insert(xyz_p,3,1,axis=1).T # insert value 1 to col 3
        # xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]<0),axis=1) # for all points, x < 0 -> delete behind vehicle

        # xyz_c = Transformer.transformLiDARToCamera(xyz_p)
        # xy_i = Transformer.transformCameraToImage(xyz_c)

        #TODO: (6) PointCloud가 Image에 투영된 Processed Image 시각화
        # xy_i = xy_i.astype(np.int32)
        # projectionImage = draw_pts_img(Transformer.img, xy_i[0,:], xy_i[1,:])   
        if not len(Transformer.lidar_pc) == 0:
            # print("lidar_pc", len(Transformer.lidar_pc))
            lidar_p = Transformer.lidar_pc[:, 0:3]
            lidar_p = np.insert(lidar_p,3,1,axis=1).T
            lidar_p = Transformer.vehicleToCameraMat.dot(lidar_p) # 4x4
            lidar_p = np.delete(lidar_p, 3, axis=0)
            lidar_xy = Transformer.transformCameraToImage(lidar_p)
            lidar_xy = lidar_xy.astype(np.int32)
            
        if not len(Transformer.radar_pc) ==0 :
            radar_p = Transformer.radar_pc[:, 0:3]
            print("radar to vehicle",radar_p)
            radar_p = np.insert(radar_p,3,1,axis=1).T
            radar_p = Transformer.vehicleToCameraMat.dot(radar_p) # 4x4
            print("vehicle to camera", radar_p)
            radar_p = np.delete(radar_p, 3, axis=0)
            radar_xy = Transformer.transformCameraToImage(radar_p)
            print("after image cal",radar_xy)
            exit(1)
            radar_xy = radar_xy.astype(np.int32)
        if Transformer.img_status == False :
            continue
        projectionImage = draw_pts_img(Transformer.img, lidar_xy[0,:], lidar_xy[1,:],(0,255,0))
        projectionImage = draw_pts_img(projectionImage, radar_xy[0,:], radar_xy[1,:],(255,0,0))
        cv2.imshow("LidartoCameraProjection", projectionImage)
        cv2.waitKey(1)


