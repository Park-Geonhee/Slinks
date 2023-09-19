#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_dijkstra_path_1 은 Mgeo 데이터를 이용하여 시작 Node 와 목적지 Node 를 지정하여 Dijkstra 알고리즘을 적용하는 예제 입니다.
# 사용자가 직접 지정한 시작 Node 와 목적지 Node 사이 최단 경로 계산하여 global Path(전역경로) 를 생성 합니다.
# 시작 Node 와 목적지 Node 는 Rviz 의 goal pose / initial pose 두 기능을 이용하여 정의합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. 시작 Node 와 종료 Node 정의
# 3. weight 값 계산
# 4. Dijkstra Path 초기화 로직
# 5. Dijkstra 핵심 코드
# 6. node path 생성
# 7. link path 생성
# 8. Result 판별
# 9. point path 생성
# 10. dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
# 11. dijkstra 이용해 만든 Global Path 정보 Publish

#TODO: (0) 필수 학습 지식
'''
# dijkstra 알고리즘은 그래프 구조에서 노드 간 최단 경로를 찾는 알고리즘 입니다.
# 시작 노드부터 다른 모든 노드까지의 최단 경로를 탐색합니다.
# 다양한 서비스에서 실제로 사용 되며 인공 위성에도 사용되는 방식 입니다.
# 전체 동작 과정은 다음과 같습니다.
#
# 1. 시작 노드 지정
# 2. 시작 노드를 기준으로 다른 노드와의 비용을 저장(경로 탐색 알고리즘에서는 비용이란 경로의 크기를 의미)
# 3. 방문하지 않은 노드들 중 가장 적은 비용의 노드를 방문
# 4. 방문한 노드와 인접한 노드들을 조사해서 새로 조사된 최단 거리가 기존 발견된 최단거리 보다 작으면 정보를 갱신
#   [   새로 조사된 최단 거리 : 시작 노드에서 방문 노드 까지의 거리 비용 + 방문 노드에서 인접 노드까지의 거리 비용    ]
#   [   기존 발견된 최단 거리 : 시작 노드에서 인접 노드까지의 거리 비용                                       ]
# 5. 3 ~ 4 과정을 반복 
# 

'''
class dijkstra_path_pub :
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)
        self.pin_point_pub = rospy.Publisher('/pinpoints_cp', PointCloud, queue_size=1)
        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.nodes=node_set.nodes
        self.links=link_set.lines

        print("nodes and links are ready")
        self.global_planner=Dijkstra(self.nodes,self.links)
        self.is_node_list = False
        self.reset_flag = 0

        self.total_total_cost = 0

        rospy.Subscriber('/pinpoint_utm_list', Float64MultiArray, self.pinpoint_list_callback)

        while True:
            if self.is_node_list == True:
                print("pinpoints are ready")
                break
            else:
                pass

        #self.global_path_msg = self.calc_dijkstra_path_node(self.start_node, self.end_node)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (11) dijkstra 이용해 만든 Global Path 정보 Publish
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()
    
    def pinpoint_list_callback(self, msg):
        pinpoints_cp = PointCloud()
        pinpoints_cp.header.frame_id='map'

        self.total_total_cost = 0
        self.target_node_list = []
        self.pinpoint_list = msg.data
        pinpoint_list_set = []
        for i in range(len(self.pinpoint_list)//2):
            pinpoint_list_set.append([self.pinpoint_list[i*2+1], self.pinpoint_list[i*2]])
            
        # make nearest nodes list from each pinpoint
        for pinpoint in pinpoint_list_set:
            min_dist = 21e8
            nearest_node = None
            for node_idx, node in self.nodes.items():
                dist = pow(pinpoint[0] - node.point[0], 2) + pow(pinpoint[1] - node.point[1], 2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_node = node_idx
            buf = Point32()
            buf.x, buf.y, buf.z = node.point[0], node.point[1], node.point[2]
            pinpoints_cp.points.append(buf)
            self.target_node_list.append(nearest_node)
        self.is_node_list = True

        print(self.target_node_list)
        self.global_path_msg.poses = []
        for i in range(len(self.target_node_list)-1):
            now_path = Path()
            now_path = self.calc_dijkstra_path_node(self.target_node_list[i], self.target_node_list[i+1])
            self.global_path_msg.poses.extend(now_path.poses)

        self.pin_point_pub.publish(pinpoints_cp)
        print(f"total cost : {self.total_total_cost}")

    def calc_dijkstra_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'

        '''
        print("node list")
        for node in path['node_path']:
            print(node)

        print("link likst")
        for link in path['link_path']:
            print(link)
        '''

        self.total_total_cost += path['cost']

        for point in path['point_path']:
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(point[0])
            read_pose.pose.position.y = float(point[1])
            read_pose.pose.position.z = float(point[2])
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        #TODO: (3) weight 값 계산
        '''
        # weight 값 계산은 각 Node 에서 인접 한 다른 Node 까지의 비용을 계산합니다.
        # 계산된 weight 값 은 각 노드간 이동시 발생하는 비용(거리)을 가지고 있기 때문에
        # Dijkstra 탐색에서 중요하게 사용 됩니다.
        # weight 값은 딕셔너리 형태로 사용 합니다.
        # 이중 중첩된 딕셔너리 형태로 사용하며 
        # Key 값으로 Node의 Idx Value 값으로 다른 노드 까지의 비용을 가지도록 합니다.
        # 아래 코드 중 self.find_shortest_link_leading_to_node 를 완성하여 
        # Dijkstra 알고리즘 계산을 위한 Node와 Node 사이의 최단 거리를 계산합니다.

        '''
        # 초기 설정
        weight = dict() 
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                weight[from_node_id][to_node.idx] = min_cost           

        return weight

    def find_shortest_link_leading_to_node(self, from_node,to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        #TODO: (3) weight 값 계산
        '''
        # 최단거리 Link 인 shortest_link 변수와
        # shortest_link 의 min_cost 를 계산 합니다.
        '''
        connected_links = from_node.get_to_links()
        shortest_link = None
        min_cost = 21e8
        for link in connected_links:
            if link.to_node != to_node : continue
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost
        
    def find_nearest_node_idx(self, distance, s):        
        idx_list = list(self.nodes.keys())
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx): 
        #TODO: (4) Dijkstra Path 초기화 로직
        # s 초기화         >> s = [False] * len(self.nodes)
        # from_node 초기화 >> from_node = [start_node_idx] * len(self.nodes)

        s = dict()
        from_node = dict() 
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx

        s[start_node_idx] = True
        distance =copy.deepcopy(self.weight[start_node_idx])

        #TODO: (5) Dijkstra 핵심 코드
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True            
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx

        #TODO: (6) node path 생성
        tracking_idx = end_node_idx
        node_path = [end_node_idx]
        
        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)     

        node_path.reverse()

        #TODO: (7) link path 생성
        link_path = []
        total_cost = 0
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
            link_path.append(shortest_link.idx)
            total_cost += min_cost
        
        #TODO: (8) Result 판별
        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[], 'cost' : total_cost}

        #TODO: (9) point path 생성
        point_path = []        
        for i, link_id in enumerate(link_path):
            
            #if link_id.find('-') != -1: #링크 묶음인 경우 idx에 '-'를 포함하고 있음 ex) A219BS010403-A219BS010405
            #    links_sharing_from_node = self.links[link_id].get_from_node_sharing_links()
            #    for link_sharing_from_node in links_sharing_from_node:
            #        if link_sharing_from_node.idx.find('-') != -1: continue
            #       link_id = link_sharing_from_node.idx
            if link_id.find('-') != -1:
                link = self.links[link_id]
                for point in link.points:
                    point_path.append([point[0], point[1], 0])
            else: #change lane link
                links = link_id.split('-')
                from_link = self.links[links[0]]
                to_link = self.links[links[-1]]
                len_from_link = len(from_link.points)
                len_to_link = len(to_link.points)

                # End Point 까지의 길이를 Point 간 간격으로 나눠 필요한 Point 의 수를 계산한다.
                # 계산된 Point 의 숫자 만큼 X 좌표를 생성한다.

                # five points of from_link 
                for j in range(5):
                    point_path.append([from_link.points[j][0],from_link.points[j][1],0])

                # points of third-order curve
                start_point_num = 5
                end_point_num = 20
                lane_change_path = self.get_lane_chage_path(from_link, to_link, start_point_num, end_point_num)
                point_path.extend(lane_change_path)
                # remains points of to_link
                for j in range(end_point_num, len_to_link):
                    point_path.append([to_link.points[j][0],to_link.points[j][1],0])
                
        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path, 'cost' : total_cost}


    def get_lane_chage_path(self, from_link, to_link, start_point_num, end_point_num):
        change_start_point = from_link.points[0]
        change_start_next_point = from_link.points[start_point_num]
        end_point_num = min(end_point_num, len(to_link.points)-1)
        change_end_point = to_link.points[end_point_num]

        translation = [change_start_point[0], change_start_point[1]]
        theta       = atan2(change_start_next_point[1]-change_start_point[1], change_start_next_point[0]-change_start_point[0])
        trans_matrix = np.array([   [ cos(theta), -sin(theta), translation[0]],
                                    [ sin(theta),  cos(theta), translation[1]],
                                    [          0,             0,              1] ])
        det_trans_matrix = np.linalg.inv(trans_matrix)

        world_end_point=np.array([[change_end_point[0]],[change_end_point[1]],[1]])
        local_end_point=det_trans_matrix.dot(world_end_point)
        waypoints_x=[]
        waypoints_y=[]
        x_interval = 0.5 # 생성할 Path 의 Point 간격을 0.5 로 한다.
        x_start=0
        x_end=local_end_point[0][0]

        y_start = 0.0
        y_end = local_end_point[1][0]

        x_num = x_end/x_interval

        for i in range(x_start,int(x_num)) : 
            waypoints_x.append(i*x_interval)

        a,b,c,d = -2 * y_end / pow(x_end, 3), 3 * y_end / pow(x_end, 2), 0, 0

        for i in waypoints_x:
            result = a * pow(i, 3) + b * pow(i,2) + c * i + d
            waypoints_y.append(result)

        out_points = []
        for i in range(0,len(waypoints_y)) :
            local_result = np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
            global_result = trans_matrix.dot(local_result)
            out_points.append([global_result[0][0],global_result[1][0],0])
        return out_points
    
if __name__ == '__main__':
    
    dijkstra_path_pub = dijkstra_path_pub()