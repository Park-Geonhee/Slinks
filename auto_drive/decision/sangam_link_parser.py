#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from lib.mgeo.class_defs import *
from std_msgs.msg import String, Bool, Float64
from ssafy_ad.msg import custom_link_parser

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

class LinkParser:
    def __init__(self):
        rospy.init_node('LinkParser', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/link_path", String, self.link_path_callback)
        rospy.Subscriber("/node_path", String, self.node_path_callback)
        
        self.link_info_pub = rospy.Publisher("link_info", custom_link_parser, queue_size=1)

        # get Mgeo data
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)
        
        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.is_odom = False
        self.is_node_path = False
        self.is_link_path = False
        self.is_node_path_set  = False
        self.is_link_path_set  = False

        # global path 생성 시 만들어지는 node_path와 link_path를 사용해야 한다. 
        while True:
            if self.is_odom == True and self.is_node_path == True and self.is_link_path == True:
                
                self.node_path_list = self.node_path.split(' ')
                self.node_path_list.pop(0)
                self.is_node_path_set = True

                self.link_path_list = self.link_path.split(' ')
                self.link_path_list.pop(0)
                self.is_link_path_set = True
                break

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # get data
            current_link = self.find_current_link()
            current_link_data = self.links[current_link.idx]
            stop_line_point, is_on_stop_line, next_drive = self.find_stop_line(current_link_data)
            possible_lattice_pathes = self.find_possible_lattice_pathes(current_link_data)
            # set msg
            link_info_msg = custom_link_parser()
            link_info_msg.link_idx = current_link.idx
            link_info_msg.stop_line_point = stop_line_point
            link_info_msg.is_on_stop_line = is_on_stop_line
            link_info_msg.possible_lattice_pathes = possible_lattice_pathes
            link_info_msg.next_drive = next_drive

            # publish
            self.link_info_pub.publish(link_info_msg)

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
    def link_path_callback(self, msg):
        self.is_link_path = True
        self.link_path = msg.data

    def node_path_callback(self, msg):
        self.is_node_path = True
        self.node_path = msg.data
        
    # node_path에 포함된 node들 중 가장 가까운 3개의 노드를 반환한다.
    def find_near_3_nodes(self):
        result = [[21e8, Node()], [21e8, Node()], [21e8, Node()]]
        
        for node_idx in self.node_path_list:
            node = self.nodes[node_idx]
            if (self.x - node.point[0]>100) or (self.y-node.point[1])>100 : continue
            ddist = pow(self.x - node.point[0],2) + pow(self.y - node.point[1],2)
            if ddist < result[2][0]:
                result[2] = [ddist, node]
                result.sort(key=lambda x:x[0])
        return result

    # 찾은 3개의 노드에 연결된 링크들 중 현재 위치와 가장 가까운 포인트를 가지고 있는 링크를 반환한다.
    def find_current_link(self):
        # 1. find nearest node
        near_nodes = self.find_near_3_nodes()

        # 2. get link connected to found node
        connected_links = []
        for i in range(len(near_nodes)):        
            connected_links += near_nodes[i][1].to_links + near_nodes[i][1].from_links

        # 3. find nearest link with current position
        current_link = Link()
        min_ddist = 21e8
        for link in connected_links:
            for point in link.points:
                ddist = pow(self.x - point[0],2) + pow(self.y - point[1],2)
                if ddist < min_ddist:
                    min_ddist = ddist
                    current_link = link

        return current_link
    '''
    def find_stop_line(self, current_link):
            current_to_node = current_link.get_to_node()
            stop_line_point = current_to_node.point

            is_on_stop_line = current_to_node.on_stop_line
            if is_on_stop_line == False:
                next_to_link = current_to_node.to_links
                if (len(next_to_link) == 1) and (next_to_link[0].get_to_node().on_stop_line):
                    next_to_node = next_to_link[0].get_to_node()
                    is_on_stop_line = next_to_node.on_stop_line
                    stop_line_point = next_to_node.point
                else:
                    pass

            return stop_line_point, is_on_stop_line
    '''
    # 정지선의 위치 찾기. acc_with_tl에서 신호등 데이터와 조합하여 사용.
    def find_stop_line(self, current_link):
        current_to_node = current_link.get_to_node()
        stop_line_point = current_to_node.point
        next_drive = " "

        current_link_idx = current_link.idx
        current_link_num = 0
        
        # link path 에서 현재 링크가 몇 번째 인덱스인지 탐색
        for num, link in enumerate(self.link_path_list):
            if link == current_link_idx:
                current_link_num = num
                break

        # 현재 링크에서 정지선이 있는 지 확인
        is_on_stop_line = current_to_node.on_stop_line
        # 정지선이 있으면
        if is_on_stop_line:
            # 경로 상 다음 링크가 있는지 확인
            try:
                next_to_link_idx = self.link_path_list[current_link_num + 1]
            # 없으면 현재 링크의 정지선만 바로 리턴
            except:
                return stop_line_point, is_on_stop_line, next_drive
            # 있으면 다음 링크의 주행 정보를 정지선에 같이 포함
            next_to_link= self.links[next_to_link_idx]
            next_drive = next_to_link.related_signal
        
        # 정지선이 없으면
        # 위의 작업을 다음 번 노드까지 확인하면서 한번 더 수행해준다.
        else:
            try:
                next_to_link_idx = self.link_path_list[current_link_num + 1]
                next_toto_link_idx = self.link_path_list[current_link_num + 2]
            except:
                return stop_line_point, is_on_stop_line, next_drive
            next_to_link = self.links[next_to_link_idx]

            next_to_node = next_to_link.get_to_node()
            is_on_stop_line = next_to_node.on_stop_line
            stop_line_point = next_to_node.point

            next_toto_link = self.links[next_toto_link_idx]
            next_drive = next_toto_link.related_signal

        if next_drive == None:
            next_drive = " "
        return stop_line_point, is_on_stop_line, next_drive

    # 차선 변경이 가능한 도로인지 판단한 뒤 가능하다면 회피경로를 생성한다.
    def find_possible_lattice_pathes(self, current_link):
        # 좌 3, 우 3 회피경로 중 차선 변경이 가능한지 판단
        result = [False, False, False, False, False, False]
        
        left_link = current_link.lane_ch_link_left
        right_link = current_link.lane_ch_link_right
        
        if left_link != None:
            result[2] = current_link.can_move_left_lane
            left2_link = left_link.lane_ch_link_left
            if left2_link != None:
                result[1] = left_link.can_move_left_lane
                left3_link = left_link.lane_ch_link_left
                if left3_link != None:
                    result[0] = left2_link.can_move_left_lane

        if right_link != None:
            result[3] = current_link.can_move_right_lane
            right2_link = right_link.lane_ch_link_right
            if right2_link != None:
                result[4] = right_link.can_move_right_lane
                right3_link = right_link.lane_ch_link_right
                if right3_link != None:
                    result[5] = right2_link.can_move_right_lane


        return result

if __name__ == '__main__':
    try:
        Link_Parser = LinkParser()
    except rospy.ROSInterruptException:
        pass