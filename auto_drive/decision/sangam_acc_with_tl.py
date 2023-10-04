#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,pi,sqrt,pow,atan2

from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList,GetTrafficLightStatus
from ssafy_ad.msg import custom_link_parser

import time

import numpy as np
import sys
import os
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from lib.mgeo.class_defs import *

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_object_info = False
        self.is_look_forward_point = False
        self.is_traffic_light_info = False
        self.is_link_path = False
        self.is_link_path_set = False

        self.link_info = custom_link_parser()
        self.forward_point = [1,1,1]
        self.current_position = Point()
        self.link_path = String()

        self.before_waypoint = 0
        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd=5
        self.max_lfd=30
        self.lfd_gain = 0.78
        self.target_velocity = 60

        rospy.Subscriber("/global_path", Path, self.global_path_callback )
        rospy.Subscriber(local_path_name, Path, self.path_callback )
        rospy.Subscriber("/odom", Odometry, self.odom_callback )
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback )
        rospy.Subscriber("/radar_detection", ObjectStatusList, self.object_info_callback )
        rospy.Subscriber("/link_info", custom_link_parser, self.get_link_info_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)
        rospy.Subscriber("/link_path", String, self.get_link_path_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size = 1)

        while True:
            if self.is_link_path == True:
                self.link_path = self.link_path.split(' ')
                self.link_path.pop(0)
                self.is_link_path_set = True
                break


        self.pid = pidControl()
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain = 0.5, distance_gain = 1, time_gap = 2, vehicle_length = 2.7, link_info = self.link_info)
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        
        #setting for traffic light from sim
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)
        traffic_light_set = mgeo_planner_map.light_set
        self.traffic_lights = traffic_light_set.signals

        #setting fro traffic light from cam
        

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                pass
                #rospy.loginfo('Waiting global path data')

        print("velocity planning is completed")
        
        
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_traffic_light_info == True and self.is_link_path_set == True:
                # 가져오는 신호등 데이터가 경로 상에 있는 신호등인지 확인하는 작업. 
                tl_idx = self.traffic_light_info.trafficLightIndex
                for tl_link in self.traffic_lights[tl_idx].link_id_list:
                    if tl_link in self.link_path: 
                        self.tl_info = self.traffic_light_info

            if self.is_path == True and self.is_odom == True and self.is_status == True:

                # global_obj,local_obj
                result = self.calc_vaild_obj([self.current_position.x,self.current_position.y,self.vehicle_yaw],self.object_data)
                
                global_npc_info = result[0] 
                local_npc_info = result[1] 
                global_ped_info = result[2] 
                local_ped_info = result[3] 
                global_obs_info = result[4] 
                local_obs_info = result[5] 

                global_tl_info = result[6]
                local_tl_info = result[7]

                self.current_waypoint = self.get_current_waypoint([self.current_position.x,self.current_position.y],self.global_path)
                try:
                    self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                except: pass

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0

                self.adaptive_cruise_control.link_info = self.link_info
                self.adaptive_cruise_control.check_object(self.path ,global_npc_info, local_npc_info
                                                                    ,global_ped_info, local_ped_info
                                                                    ,global_obs_info, local_obs_info
                                                                    ,global_tl_info, local_tl_info)
                self.target_velocity = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, local_obs_info, local_tl_info,
                                                                                                        self.status_msg.velocity.x, self.target_velocity/3.6)

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data 

    def traffic_light_callback(self,msg):
        self.traffic_light_info = msg
        self.is_traffic_light_info = True

    def get_link_info_callback(self, msg):
        self.link_info = msg

    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1     

        ego_pose_x = ego_status[0]
        ego_pose_y = ego_status[1]

        for i,pose in enumerate(global_path.poses):
            dx = ego_pose_x - pose.pose.position.x
            dy = ego_pose_y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i

        if currnet_waypoint == -1:
            currnet_waypoint = min(self.before_waypoint + 1, len(global_path.poses)-1)

        self.before_waypoint = currnet_waypoint

        return currnet_waypoint

    def get_link_path_callback(self, msg):
        self.is_link_path = True
        self.link_path = msg.data


    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []
        global_tl_info = [] 
        local_tl_info = []

        #translation
        tmp_theta=ego_heading
        tmp_translation=[ego_pose_x, ego_pose_y]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                        [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                        [0             ,               0,                  1]])
        tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                            [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                            [0,0,1]])

        #traffic light translation
        cur_traffic_light_index = None
        if self.is_traffic_light_info == True:
            cur_traffic_light_index = self.traffic_light_info.trafficLightIndex
            
            #traffic_lihgt_set에서 현재 도로에서의 신호등 찾기
            for idx, traffic_light in self.traffic_lights.items():
                if idx != cur_traffic_light_index : continue
                self.cur_traffic_light = traffic_light

            stop_line_point = self.link_info.stop_line_point
            global_result = np.array([[stop_line_point[0]],[stop_line_point[1]],[1]])
            local_result = tmp_det_t.dot(global_result)

            # tl_info는 전역 경로 상에 존재하는 링크의 신호등인지 판단 후 생성되기 때문에
            # 출발지의 신호등 정보가 undetected인 경우 오류가 발생하므로 try catch 구문을 사용한다. 
            try : 
                global_tl_info = [[stop_line_point[0], stop_line_point[1]], 
                                self.tl_info.trafficLightType,self.tl_info.trafficLightStatus]
                
                local_tl_info = [[local_result[0][0], local_result[1][0]],
                                self.tl_info.trafficLightType, self.tl_info.trafficLightStatus]
            except:
                global_tl_info = [[stop_line_point[0], stop_line_point[1]], 
                                self.traffic_light_info.trafficLightType,self.traffic_light_info.trafficLightStatus]
                
                local_tl_info = [[local_result[0][0], local_result[1][0]],
                                self.traffic_light_info.trafficLightType, self.traffic_light_info.trafficLightStatus]


        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian + 1
        if num_of_object > 0:

            #npc vehicle translation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result=np.array([[ped_list.position.x],[ped_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_ped_info.append([ped_list.type,ped_list.position.x,ped_list.position.y,ped_list.velocity.x])
                    local_ped_info.append([ped_list.type,local_result[0][0],local_result[1][0],ped_list.velocity.x])

            #obs translation
            for obs_list in self.all_object.obstacle_list:
                global_result=np.array([[obs_list.position.x],[obs_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_obs_info.append([obs_list.type,obs_list.position.x,obs_list.position.y,obs_list.velocity.x])
                    local_obs_info.append([obs_list.type,local_result[0][0],local_result[1][0],obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info, global_tl_info, local_tl_info

    def calc_pure_pursuit(self,):

        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정
        
        # 차량 속도에 비례하여 전방주시거리(Look Forward Distance) 가 변하는 수식을 구현 합니다.
        # 이때 'self.lfd' 값은 최소와 최대 값을 넘어서는 안됩니다.
        # "self.min_lfd","self.max_lfd", "self.lfd_gain" 을 미리 정의합니다.
        # 최소 최대 전방주시거리(Look Forward Distance) 값과 속도에 비례한 lfd_gain 값을 직접 변경해 볼 수 있습니다.
        # 초기 정의한 변수 들의 값을 변경하며 속도에 비례해서 전방주시거리 가 변하는 advanced_purepursuit 예제를 완성하세요.
        # 
        self.lfd = self.lfd_gain * self.status_msg.velocity.x
        if self.lfd < self.min_lfd :
            self.lfd = self.min_lfd
        if self.lfd > self.max_lfd:
            self.lfd = self.max_lfd

        #rospy.loginfo(self.lfd)        
        
        vehicle_position=self.current_position
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성
        
        # Pure Pursuit 알고리즘을 실행 하기 위해서 차량 기준의 좌표계가 필요합니다.
        # Path 데이터를 현재 차량 기준 좌표계로 좌표 변환이 필요합니다.
        # 좌표 변환을 위한 좌표 변환 행렬을 작성합니다.
        # Path 데이터를 차량 기준 좌표 계로 변환 후 Pure Pursuit 알고리즘 중 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 찾습니다.
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 이용하여 조향 각도를 계산하게 됩니다.
        # 좌표 변환 행렬을 이용해 Path 데이터를 차량 기준 좌표 계로 바꾸는 반복 문을 작성 한 뒤
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 계산하는 로직을 작성 하세요.

                # ## 학습 내용 ##
        # 변환 행렬 계산: 두 좌표계 간의 변환 행렬을 계산해야 합니다. 이 행렬은 회전, 이동 및 크기 조절 변환을 포함합니다. 주어진 정보에 따라 이 행렬을 결정할 수 있습니다.
        # 변환 공식 적용: 변환 행렬을 사용하여 원래 좌표를 새 좌표계로 변환합니다. 일반적으로 행렬과 좌표를 행렬-벡터 곱셈으로 곱하는 방식을 사용합니다.
        
        # | x' |   | cos(θ) -sin(θ) tx |   | x |
        # | y' | = | sin(θ)  cos(θ) ty | * | y |
        # | 1  |   |   0       0    1  |   | 1 |
        
        # 여기서 (x, y)는 원래 좌표이고, (x', y')는 새 좌표계로 변환된 좌표입니다. θ는 회전 각도이며, tx와 ty는 이동 변환입니다.

        # /Ego_topic 토픽의 heading 값(deg)
        # math method ~ angle : radian
        # rad to deg : * 180 / pi
        # deg to rad : * pi / 180
        
        # self.ego_status = EgoVehicleStatus()
        beta = self.status_msg.heading
        beta = beta * pi / 180
        trans_matrix = np.array([   [cos(beta), -sin(beta), translation[0]],
                                    [sin(beta),  cos(beta), translation[1]],
                                    [ 0       , 0         , 1             ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point = i

            global_path_point = [ path_point.pose.position.x, path_point.pose.position.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0]>0 :
                dis = sqrt(local_path_point[0]*local_path_point[0] + local_path_point[1]*local_path_point[1])
                if dis >= self.lfd :
                    self.forward_point = local_path_point
                    self.is_look_forward_point = True
                    break
        
        #TODO: (4) Steering 각도 계산

        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        sin_alpha = self.forward_point[1]/sqrt(self.forward_point[0]*self.forward_point[0] + self.forward_point[1]*self.forward_point[1])
        theta = atan2( 2 * self.vehicle_length*sin_alpha ,self.lfd)
        steering = theta
        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        
        # 종방향 제어를 위한 PID 제어기는 현재 속도와 목표 속도 간 차이를 측정하여 Accel/Brake 값을 결정 합니다.
        # 각 PID 제어를 위한 Gain 값은 "class pidContorl" 에 정의 되어 있습니다.
        # 각 PID Gain 값을 직접 튜닝하고 아래 수식을 채워 넣어 P I D 제어기를 완성하세요.

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        pidValue = p_control + self.i_control + d_control

        output = pidValue
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        # self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            car_max_speed = global_path.poses[i].pose.position.z / 3.6
            out_vel_plan.append(car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            car_max_speed = global_path.poses[i].pose.position.z / 3.6
            for box in range(-point_num, point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            # 도로의 곡률 반경을 계산하기 위한 수식입니다.
            # Path 데이터의 좌표를 이용해서 곡선의 곡률을 구하기 위한 수식을 작성합니다.

            aMat = np.array(x_list)
            bMat = np.array(y_list) 

            # 원의 좌표를 구하는 행렬 계산식, 최소 자승법을 이용하는 방식 등 곡률 반지름을 구하기 위한 식을 적용 합니다.
            aMatTrans = np.transpose(aMat)

            try:
                resMat = np.linalg.inv(aMatTrans.dot(aMat)).dot(aMatTrans).dot(bMat)
                #resMat = np.linalg.inv(((aMatTrans.dot(aMat)).dot(aMatTrans)).dot(bMat))
                # 적용한 수식을 통해 곡률 반지름 "r" 을 계산합니다.
                # temp_val = resMat[0]*resMat[0] + resMat[1]*resMat[1] - resMat[2]
                r = sqrt(resMat[0]*resMat[0] + resMat[1]*resMat[1] - resMat[2])
                

                #TODO: (7) 곡률 기반 속도 계획
                # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
                # 평평한 도로인 경우 최대 속도를 계산합니다. 
                # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.
                v_max =sqrt(r * 9.81 * self.road_friction ) + 1.5

                if v_max > car_max_speed:
                    v_max = car_max_speed
                out_vel_plan.append(v_max)
            except:
                out_vel_plan.append(car_max_speed)


        for i in range(len(global_path.poses) - point_num, len(global_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length, link_info):
        self.npc_vehicle=[False,0]
        self.object=[False,0]
        self.Person=[False,0]
        self.tl =[False,0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length
        self.link_info = link_info
        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0
        
    def check_object(self,ref_path, global_npc_info, local_npc_info, 
                                    global_ped_info, local_ped_info, 
                                    global_obs_info, local_obs_info,
                                    global_tl_info, local_tl_info):
        #TODO: (8) 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)
        
        # 주행 경로 상의 장애물의 유무를 파악합니다.
        # 장애물이 한개 이상 있다면 self.object 변수의 첫번째 값을 True 로 둡니다.
        # 장애물의 대한 정보는 List 형식으로 self.object 변수의 두번째 값으로 둡니다.
        # 장애물의 유무 판단은 주행 할 경로에서 얼마나 떨어져 있는지를 보고 판단 합니다.
        # 아래 예제는 주행 경로에서 Object 까지의 거리를 파악하여 
        # 경로를 기준으로 2.5 m 안쪽에 있다면 주행 경로 내 장애물이 있다고 판단 합니다.
        # 주행 경로 상 장애물이 여러게 있는 경우 가장 가까이 있는 장애물 정보를 가지도록 합니다.
        
        # 주행 경로 상 보행자 유무 파악
        min_rel_distance=float('inf')
        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :      
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]                    
                        dis = sqrt(pow(global_ped_info[i][1] - path.pose.position.x, 2) + pow(global_ped_info[i][2] - path.pose.position.y, 2))
                        if dis < 2.5:                            
                            rel_distance = sqrt(pow(local_ped_info[i][1], 2) + pow(local_ped_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person=[True,i]        
 
        
        # 주행 경로 상 NPC 차량 유무 파악
        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dis = sqrt(pow(global_npc_info[i][1] - path.pose.position.x, 2) + pow(global_npc_info[i][2] - path.pose.position.y, 2))
                        if dis < 2.5:
                            rel_distance = sqrt(pow(local_npc_info[i][1], 2) + pow(local_npc_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.npc_vehicle=[True,i]
        
        
        # 주행 경로 상 Obstacle 유무 파악
        # acc 예제는 주행 중 전방에 차량에 속도에 맞춰 움직이도록 하는 Cruise Control
        # 예제 이기 때문에 정적 장애물(Obstacle) 의 정보는 받지 않는게 좋습니다.
        # 정적 장애물은 움직이지 않기 때문에 Cruise Control 알고리즘 상
        # 정적 장애물을 만나게 되면 속도가 0인 정적 장애물 바로 뒤에 정지하게 됩니다.
        if len(global_obs_info) > 0 :            
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
                        dis = sqrt(pow(global_obs_info[i][1] - path.pose.position.x, 2) + pow(global_obs_info[i][2] - path.pose.position.y, 2))
                        if dis < 2.5:
                            rel_distance = sqrt(pow(local_obs_info[i][1], 2) + pow(local_obs_info[i][2], 2))               
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                #self.object=[True,i] 

        # 주행 경로 상 Traffic Light 유무 파악
        if global_tl_info:
            for path in ref_path.poses :        
                dis = sqrt(pow(global_tl_info[0][0] - path.pose.position.x, 2) + pow(global_tl_info[0][1] - path.pose.position.y, 2))
                if dis < 2.5:
                    rel_distance = sqrt(pow(local_tl_info[0][0], 2) + pow(local_tl_info[0][1], 2))           
                    if rel_distance < min_rel_distance:
                        min_rel_distance = rel_distance
                        self.tl=[True,0]
    
    def get_target_velocity(self, local_npc_info, local_ped_info, local_obs_info, local_tl_info, ego_vel, target_vel): 
        #TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
        out_vel =  target_vel
        default_space = 8
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain

        if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle   
            #print("ACC ON NPC_Vehicle")         
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
            vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration      

        if self.Person[0] and len(local_ped_info) != 0: #ACC ON_Pedestrian
            #print("ACC ON Pedestrian")
            Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
            vel_rel = (Pedestrian[2] - ego_vel)              
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration
   
        if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle     
            #print("ACC ON Obstacle")                    
            Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
            vel_rel = (Obstacle[2] - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration           

            '''
            # if you use this block for Obstacle,
            # you have to use line 514  ~ self.object=[True,i]
            # and also, you have to comment out right above block (line 557 ~ line 566)
            if self.object[0] and len(local_obs_info) != 0:
                for i in range(len(local_obs_info)):
                    if abs(local_obs_info[i][2]) > 1.5 : continue
                    # print("ACC ON Obstacle")
                    dis_rel = sqrt(pow(local_obs_info[i][1], 2) + pow(local_obs_info[i][2], 2))
                    vel_rel = local_obs_info[i][3] - ego_vel
                    dis_safe = ego_vel* time_gap + default_space
                    acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)
                    out_vel = min(out_vel, ego_vel + acceleration)
            '''

        if self.tl[0]: #ACC ON_traffic_light
            # 정지선이 있으면
            if self.link_info.is_on_stop_line:
                
                # 현재 링크에서의 신호등 정보가 없는 경우는 넘어감
                if local_tl_info[0][0] == 0 : pass

                # 좌회전일 경우
                elif self.link_info.next_drive.find("left") != -1:
                    if (((local_tl_info[2]>>5)&1) == 0):
                        dis_safe = ego_vel * time_gap*0.25 + default_space - 2
                        dis_rel = sqrt(pow(local_tl_info[0][0],2) + pow(local_tl_info[0][1],2))            
                        vel_rel=(0 - ego_vel)                        
                        acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)
                        out_vel = ego_vel + acceleration
                # 좌회전이 아닐 경우
                else:
                    if ((local_tl_info[2]&1) ==1):
                        dis_safe = ego_vel * time_gap*0.25 + default_space - 2
                        dis_rel = sqrt(pow(local_tl_info[0][0],2) + pow(local_tl_info[0][1],2))            
                        vel_rel=(0 - ego_vel)                        
                        acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)
                        out_vel = ego_vel + acceleration
            '''
            if (local_tl_info[2] == 1 or (local_tl_info[2]== 33) or (local_tl_info[2]== 5)) and local_tl_info[0][0]>0 and self.is_on_stop_line: 
                dis_safe = ego_vel * time_gap*0.25 + default_space - 2
                dis_rel = sqrt(pow(local_tl_info[0][0],2) + pow(local_tl_info[0][1],2))            
                vel_rel=(0 - ego_vel)                        
                acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)
                out_vel = ego_vel + acceleration
            '''
            
        
        out_vel = min(out_vel, target_vel) * 3.6

        
        return out_vel


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
