#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
import pyproj

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import PointCloud
from morai_msgs.msg import GPSMessage

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class astar_path_pub :
    def __init__(self):
        rospy.init_node('astar_path_pub', anonymous=True)
        #rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)
        self.pin_point_pub = rospy.Publisher('/pinpoints_cp', PointCloud, queue_size=1)
        self.road_point_pub = rospy.Publisher('/road_point', String, queue_size=10)

        self.node_path_pub = rospy.Publisher('/node_path', String, queue_size = 1)
        self.link_path_pub = rospy.Publisher('/link_path', String, queue_size = 1)


        
        self.proj_latlon = pyproj.Proj(proj='latlong', datum='WGS84')
        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=True)

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
        self.global_planner=Astar(self.nodes,self.links)
        self.is_node_list = False
        self.is_global_path = False
        self.reset_flag = 0

        self.total_total_cost = 0

        rospy.Subscriber('/pinpoint_utm_list', Float64MultiArray, self.pinpoint_list_callback)

        while True:
            if self.is_node_list == True:
                print("pinpoints are ready")
                break
            else:
                pass

        #self.global_path_msg = self.calc_astar_path_node(self.start_node, self.end_node)
    

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_global_path == False : continue
            self.global_path_pub.publish(self.global_path_msg)
            #self.publish_road_points(self.global_path_msg)
            rate.sleep()
    '''
    def navsat_callback(self, gps_msg):

        #self.lat = gps_msg.latitude
        #self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True
    '''
    '''
    def publish_road_points(self, path_msg):
        road_points_str = ""
        for pose in path_msg.poses:
            x, y = pose.pose.position.x+313008.5581900857, pose.pose.position.y+4161698.628368007
            #lon, lat = transform(self.proj_UTM, self.proj_WGS84, pp_utm_x, pp_utm_y)
            #lon_W, lat_W = self.proj_UTM(x, y, inverse = True)
            #transformer = pyproj.Transformer.from_proj(self.proj_UTM, self.proj_latlon, always_xy=True)
            #lon, lat = self.proj_UTM(x, y, inverse = True)
            road_points_str += "{}, {}; ".format(x, y)
        self.road_point_pub.publish(road_points_str)
        313008.5581900857
        4161698.628368007
    '''
    def publish_road_points(self, path_msg):
        road_points_str = ""
        
        # 짝수 인덱스의 좌표만 선택하여 사용합니다.
        for idx, pose in enumerate(path_msg.poses):
            if idx % 30 == 0:
                x, y = pose.pose.position.x+313008.5581900857, pose.pose.position.y+4161698.628368007
                road_points_str += "{:.6f}, {:.6f}; ".format(x, y)
        
        self.road_point_pub.publish(road_points_str)

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

        self.global_path_msg.poses = []
        for i in range(len(self.target_node_list)-1):
            now_path = Path()
            now_path = self.calc_astar_path_node(self.target_node_list[i], self.target_node_list[i+1])
            self.global_path_msg.poses.extend(now_path.poses)

        self.pin_point_pub.publish(pinpoints_cp)
        print(f"total cost : {self.total_total_cost}")
        self.publish_road_points(self.global_path_msg)
        self.is_global_path = True


    def calc_astar_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) astar 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'

        node_msg = ''
        for node in path['node_path']:
            node_msg += f" {node}"
        self.node_path_pub.publish(node_msg)

        link_msg = ''
        for link in path['link_path']:
            idx = link.find('-')
            if idx != -1:
                link_msg += f" {link[:idx]}"
                link_msg += f" {link[idx+1:]}"
            else : link_msg += f" {link}"
        self.link_path_pub.publish(link_msg)



        self.total_total_cost += path['cost']

        for point in path['point_path']:
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(point[0])
            read_pose.pose.position.y = float(point[1])
            read_pose.pose.position.z = float(point[2])
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        return out_path

class Astar:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    class MyNode:
        def __init__(self, parent = None, id = None, node = None):
            self.parent = parent
            self.id = id
            self.position = [node.point[0], node.point[1]]

            self.g = 0
            self.h = 0
            self.f = 0
    
    def heuristic(self, now, goal):
        disX = abs(now.position[0] - goal.position[0])
        disY = abs(now.position[1] - goal.position[1])
        return disX + disY

    def get_weight_matrix(self):
        
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
        '''
        여기서부터 Astar 구현 시작
        '''
        
        #TODO: (4) Path 초기화 로직 ?
        # s 초기화         >> s = [False] * len(self.nodes)
        # from_node 초기화 >> from_node = [start_node_idx] * len(self.nodes)
        
        #TODO: (5) Astar(다익스트라) 핵심 코드

        # 시작 노드, 끝 노드의 Node 객체 생성
        startNode = self.MyNode(None, start_node_idx, self.nodes[start_node_idx])
        endNode = self.MyNode(None, end_node_idx, self.nodes[end_node_idx])
        
        # openList, closedList 초기화
        openList = []
        closedList = []

        # openList 에 시작 노드 추가
        openList.append(startNode)

        # openList가 빌 때까지 무한루프
        while openList:
            print(len(openList))
            # 현재 노드 지정
            currentNode = openList[0]
            currentIdx = 0

            # 이미 같은 노드가 openList에 있고 F(=g+h) 값이 더 크면
            # currentNode 값을 openList 안에 있는 값으로 교체
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    currentIdx = index

            # openList 에서 제거하고 closedList 에 추가
            openList.pop(currentIdx)
            closedList.append(currentNode.id)

            # 현재 노드가 목적지면 current.position 추가
            # current의 부모로 이동
            # current 부모를 역추적하며
            # node path 경로 생성, 반복문 탈출
            if currentNode.id == endNode.id:
                path = []
                current = currentNode
                while current is not None:
                    path.append(current.id)
                    current = current.parent
                node_path = path[::-1]
                break

            # currentNode의 자식 노드 리스트 초기화 및 생성
            children = []
            for nextIdx, nextVal in self.weight[currentNode.id].items():
                if nextVal == float('inf'):
                    continue
                new_node = self.MyNode(currentNode, nextIdx, self.nodes[nextIdx])
                children.append(new_node)
            
            # 자식들 모두 검사
            for child in children:
                # 자식이 closedList에 있으면 continue
                if child.id in closedList:
                    continue
                # f, g, h 값 업데이트
                child.g = currentNode.g + self.weight[currentNode.id][child.id]
                child.h = self.heuristic(child, endNode)
                child.f = child.g + child.h

                # 자식이 openList에 있고 g값이 기존보다 더 크다면 continue
                if len([openNode for openNode in openList
                        if child == openNode and child.g > openNode.g]) > 0:
                    continue
                openList.append(child)

        #TODO: (6) node path 생성

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
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[]}

        #TODO: (9) point path 생성
        point_path = []        
        for i, link_id in enumerate(link_path):
            if link_id.find('-') == -1:
                link = self.links[link_id]
                link_max_speed = link.max_speed
                for point in link.points:
                    point_path.append([point[0], point[1], link_max_speed])
            else: #change lane link

                links = link_id.split('-')
                from_link = self.links[links[0]]
                from_link_max_speed = from_link.max_speed
                to_link = self.links[links[-1]]
                to_link_max_speed = to_link.max_speed
                len_from_link = len(from_link.points)
                len_to_link = len(to_link.points)
                

                # End Point 까지의 길이를 Point 간 간격으로 나눠 필요한 Point 의 수를 계산한다.
                # 계산된 Point 의 숫자 만큼 X 좌표를 생성한다.

                # five points of from_link 
                for j in range(5):
                    point_path.append([from_link.points[j][0],from_link.points[j][1],from_link_max_speed])

                # points of third-order curve
                start_point_num = 3
                end_point_num = 50
                lane_change_path = self.get_lane_chage_path(from_link, to_link, start_point_num, end_point_num)
                point_path.extend(lane_change_path)
                # remains points of to_link
                for j in range(end_point_num, len_to_link):
                    point_path.append([to_link.points[j][0],to_link.points[j][1],to_link_max_speed])
                
        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path, 'cost' : total_cost}



    def get_lane_chage_path(self, from_link, to_link, start_point_num, end_point_num):
        change_start_point = from_link.points[0]
        max_speed = from_link.max_speed
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
            out_points.append([global_result[0][0],global_result[1][0],max_speed])
        return out_points
    
if __name__ == '__main__':
    
    astar_path_pub = astar_path_pub()