#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
from ssafy_ad.msg import custom_link_parser
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name,ObjectStatusList, self.object_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/link_info", custom_link_parser, self.get_link_info_callback)
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=10)

        self.ref_path = Path()  # Reference path
        self.ego_status = EgoVehicleStatus()  # Ego vehicle's current status
        self.link_info = custom_link_parser()

        self.is_path = False
        self.is_status = False
        self.is_obj = False
        self.is_link_info = False

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_obj:
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    #TODO: (7) lattice 경로 메세지 Publish
                    if lattice_path_index != -1:
                        self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                    else:
                        self.lattice_path_pub.publish(self.local_path)
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        is_crash = False
        for obstacle in object_data.obstacle_list:  # 가정: EgoVehicleStatus에 obstacle_list가 있다고 가정
            for path in ref_path.poses:
                dis = sqrt(pow(obstacle.position.x - path.pose.position.x, 2) + pow(obstacle.position.y - path.pose.position.y, 2))
                if dis < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                        is_crash = True
                        break


        return is_crash

    def collision_check(self, object_data, out_path):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        '''
        # 충돌 회피 경로를 생성 한 이후 가장 낮은 비용의 경로를 선택 합니다.
        # lane_weight 에는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
        # 이 중 장애물이 있는 차선에는 가중치 값을 추가합니다.
        # 모든 Path를 탐색 후 가장 비용이 낮은 Path를 선택하게 됩니다.
        # 장애물이 존제하는 차선은 가중치가 추가 되어 높은 비용을 가지게 되기 떄문에 
        # 최종적으로 가장 낮은 비용은 차선을 선택 하게 됩니다. 

        '''
        
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path 
        for i in range(len(lane_weight)):
            if self.link_info.possible_lattice_pathes[i] == False:
                lane_weight[i] = int(21e8)
    
        #if self.link_info.possible_lattice_pathes[i] == False

        for obstacle in object_data.obstacle_list:
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100
        
        selected_lane = lane_weight.index(min(lane_weight))     
        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg
        
    def status_callback(self,msg): ## Vehicl Status Subscriber
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

    def get_link_info_callback(self,msg):
        self.is_link_info = True
        self.link_info = msg

    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        
        if look_distance < 20 : #min 10m   
            look_distance = 20                    

        if len(ref_path.poses) > look_distance :
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-10.5, -7.0, -3.5, 3.5, 7.0, 10.5]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                #if is_posssible_path[i] == False: continue
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
            for end_point in local_lattice_points :
            '''

            for end_point in local_lattice_points :
                waypoints_x = []
                waypoints_y = []
                x_interval = 0.5
                x_start, x_end = 0, end_point[0]
                y_start, y_end = 0, end_point[1]
                temp_out_path = Path()
                x_num = x_end / x_interval

                for i in range(x_start, int(x_num)):
                    waypoints_x.append(i * x_interval)
                a, b = -2 * y_end / pow(x_end, 3), 3 * y_end / pow(x_end, 2)

                for i in waypoints_x:
                    result = a * pow(i,3) + b * pow(i,2)
                    waypoints_y.append(result)

                for i in range(0,len(waypoints_y)) :
                    local_result = np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0.
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    temp_out_path.poses.append(read_pose)
                out_path.append(temp_out_path)

            # Add_point            
            # 3 차 곡선 경로가 모두 만들어 졌다면 이후 주행 경로를 추가 합니다.
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance * 2, add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,
                                      ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x, ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                                      [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]],
                                      [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
            
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            '''
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

            '''
            out_path = out_path[::-1]

            for i in range(len(out_path)):          
                # 동적으로 Publisher 객체 생성
                if self.link_info.possible_lattice_pathes[i] == False: continue
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1), Path, queue_size=1)
                out_path[i].header.frame_id = 'map'

                # 해당 경로를 발행
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

        print(len(out_path))
        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
