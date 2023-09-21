#!/usr/bin/env python3
#-*- coding:utf-8 -*-

# 1. get local data wrt radar
# 2. Vehicle cal
# 3. get local data wrt vehicle
# 4. Camera cal
# 5. Image cal
# 6. BBox mapping
# 7. object detect and delete multi detects
# 8. global cal
# 9. XYZ(global)+velocity put in objectStatus 
import rospy
import cv2
import torch
import numpy as np
import math
import time
import os
from morai_msgs.msg import RadarDetections, RadarDetection, ObjectStatusList, ObjectStatus
from sensor_msgs.msg import PointCloud2, CompressedImage, PointCloud
from geometry_msgs.msg import PoseArray,Pose, Point32
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
parameters_cam ={
    "WIDTH": 640, #image width
    "HEIGHT": 640, #image height
    "FOV": 90, # Field of views
    "X": 3.67, # meter
    "Y": -0.01,
    "Z": 0.51,
    "YAW": np.radians(0), # radian
    "PITCH": np.radians(0),
    "ROLL": np.radians(0)
}
PATH= "/home/leesh/catkin_ws/src/ssafy_ad/S09P22A701/seunghyuk/traffic_light"
os.chdir(PATH)
lidarPositionOffset = np.array([0, 0, -0.25 ]) # VLP16 사용해야 함
camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset  x,y,z
camera_pos = np.array([parameters_cam.get(i) for i in (["X","Y","Z"])]) + camPositionOffset    
camera_rpy = np.array([parameters_cam.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*math.pi/180,0,-90*math.pi/180])
radar_pos = np.array([parameters_radar.get(i) for i in (["X","Y","Z"])]) 
radar_rpy = np.array([parameters_radar.get(i) for i in (["ROLL","PITCH","YAW"])])  

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

def get2VehicleMat(pos,rpy):
    # sensor to vehicle matrix
    sensorRotationMat = getRotMat(rpy)
    sensorTranslationMat = np.array([pos])
    Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat,sensorTranslationMat.T),axis = 1)
    Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0,0,0,1],axis = 0)
    return Tr_sensor_to_vehicle

def get2CameraMat(r_pos, r_rpy, c_pos, c_rpy):
    radar2vehicle = get2VehicleMat(r_pos, r_rpy)
    camera2vehicle= get2VehicleMat(c_pos, c_rpy)
    vehicle2camera = inv(camera2vehicle)
    mat = vehicle2camera.dot(radar2vehicle)

    return mat

def get2ImageMat(params_cam):
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
class RadarObject:
    def __init__(self):
        rospy.Subscriber('/radar', RadarDetections, self.radar_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.object_pub = rospy.Publisher('forward_object',ObjectStatusList, queue_size=1)
        self.model =torch.hub.load(PATH, 'custom', 'yolov5s-int8.tflite',source='local')
        self.radar2VehicleMat = get2VehicleMat(radar_pos, radar_rpy)
        self.radar2CameraMat = get2CameraMat(radar_pos, radar_rpy, camera_pos, camera_rpy)
        self.camera2ImageMat = get2ImageMat(parameters_cam)
        self.width = parameters_cam["WIDTH"]
        self.height = parameters_cam["HEIGHT"]
        self.image = None
        self.radar_data = None
        self.origin_detection_list = None
        self.origin_point_list = None
        

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.image is None :
                print("get image")
                continue
            if self.radar_data is None :
                print("get radar_data")
                continue
            self.get_detection_list_wrt_vehicle()


    def radar_callback(self, msg):
        # 1. local data list wrt radar sensor
        self.radar_data = msg

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def get_detection_wrt_vehicle(self, detect):
        # 2. vehicle cal
        xyz_r = np.array([detect.position.x,detect.position.y, detect.position.z, 1])
        xyz_v = self.radar2VehicleMat.dot(xyz_r.T)

        data = ObjectStatus()
        data.unique_id = detect.detection_id
        data.position.x = xyz_v[0]
        data.position.y = xyz_v[1]
        data.position.z = xyz_v[2]
        data.velocity.x = detect.rangerate # relative speed
        return data

    def get_point_wrt_camera(self, detect):
        xyz_r = np.array([detect.position.x,detect.position.y, detect.position.z, 1])
        # 3. radar to vehicle
        # 4. vehicle to camera
        xyz_v = self.radar2CameraMat.dot(xyz_r.T)
        point = np.array([xyz_v[0],xyz_v[1],xyz_v[2],1],np.float32)
        return point
  
    def get_imageXY_from_point(self, point):
        # Camera 3d point -> image 2d point
        point_wrt_camera = point[:3]
        # 5. camera to image
        image_xy = self.camera2ImageMat.dot(point_wrt_camera)
        if image_xy[2] < 0:
            return None
        image_xy /= (image_xy[2])
        if image_xy[0] > self.width or image_xy[1] > self.height :
            return None
        image_xy = image_xy.astype(np.int32)
        return image_xy
    
    def draw_point_to_image(self, img, x, y):
        point_np = cv2.circle(img, (x,y), 3, (0,255,0), -1)
        return point_np
    
    def get_detection_list_wrt_vehicle(self):
        # 3. get local data wrt vehicle
        detection_list = ObjectStatusList()

        results = self.model(self.image) # yolov5 inference
        object_list = results.pandas().xyxy[0]
        object_cnt = len(object_list)
        check_list = [False for i in range(object_cnt)]

        # print(results.pandas().xyxy[0]["xmax"])   

        for detect in self.radar_data.detections :
            if detect.position.x == 0 and detect.position.y == 0 and detect.position.z == 0 :
                continue
            dist = (detect.position.x**2 + detect.position.y**2)**0.5
            if dist > 50 : # Too far object skip
                continue
            rate = abs(detect.position.x/detect.position.y)
            if rate < 1 :
                print("rate x/y", rate)
                continue
            # check if detection is exist and included in image frame
            point_wrt_camera = self.get_point_wrt_camera(detect) # xyz1
            image_xy = self.get_imageXY_from_point(point_wrt_camera)
            
            if image_xy is None :
                print("None because of this point")
                continue
            # it's valid, so needed to remove detections pointing same object 
            # print("column",object_list.columns) #xmin, ymin, xmax, ymax
            # print("object_list",object_list)
            is_first = False
            for i in range(object_cnt) :
                coordi = object_list.iloc[i][0:4] #xmin, ymin, xmax, ymax
                # print("coordi[0]",coordi[0])       
                if check_list[i] == True : # already mapped with radar point
                    continue
                if image_xy[0] < coordi[0] or image_xy[0] > coordi[2]:
                    continue
                if image_xy[1] < coordi[1] or image_xy[1] > coordi[3]:
                    continue
                # this BBOX include this point
                # fail detect
                name = object_list.iloc[i]["name"]
                if name in ["class2","class5","class7"]:
                    print("object name", "CAR")
                else:
                    continue
                check_list[i] = True
                is_first = True
                detection_wrt_vehicle = self.get_detection_wrt_vehicle(detect)
                detection_wrt_vehicle.name = name
                break
            if is_first == True :
                detection_list.obstacle_list.append(detection_wrt_vehicle)
                
            # projection_image = self.draw_point_to_image(self.image, image_xy[0], image_xy[1])
            # cv2.imshow("image", projection_image)
            # cv2.waitKey(1)
        self.origin_detection_list = detection_list
        self.origin_detection_list.header.stamp = time.gmtime()
        self.object_pub.publish(self.origin_detection_list)
        print("detection_list")
    
        print(self.origin_detection_list)

if __name__ == '__main__':
    rospy.init_node('radar_test',anonymous=True)
    detect = RadarObject()
    # rospy.spin()
