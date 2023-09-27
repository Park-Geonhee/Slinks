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

        while True:
            if self.is_odom == True and self.is_node_path == True and self.is_link_path == True:
                
                self.node_path = self.node_path.split(' ')
                self.node_path.pop(0)
                self.is_node_path_set = True

                self.link_path = self.link_path.split(' ')
                self.link_path.pop(0)
                self.is_link_path_set = True
                break

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            # get data
            current_link = self.find_current_link()
            current_link_data = self.links[current_link.idx]
            stop_line_point, is_on_stop_line = self.find_stop_line(current_link_data)
            possible_lattice_pathes = self.find_possible_lattice_pathes(current_link_data)

            # set msg
            link_info_msg = custom_link_parser()
            link_info_msg.link_idx = current_link.idx
            link_info_msg.stop_line_point = stop_line_point
            link_info_msg.is_on_stop_line = is_on_stop_line
            link_info_msg.possible_lattice_pathes = possible_lattice_pathes

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
        
    def find_near_3_nodes(self):
        result = [[21e8, Node()], [21e8, Node()], [21e8, Node()]]
        
        for node_idx in self.node_path:
            node = self.nodes[node_idx]
            if (self.x - node.point[0]>100) or (self.y-node.point[1])>100 : continue
            ddist = pow(self.x - node.point[0],2) + pow(self.y - node.point[1],2)
            if ddist < result[2][0]:
                result[2] = [ddist, node]
                result.sort(key=lambda x:x[0])
        return result

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
    
    def find_possible_lattice_pathes(self, current_link):
        result = [False, False, False, False, False, False]
        
        left_link = current_link.lane_ch_link_left
        right_link = current_link.lane_ch_link_right
        #print(current_link.idx)
        #print(right_link.idx)
        
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


        #print(f"in parser : {result}")
        return result

if __name__ == '__main__':
    try:
        Link_Parser = LinkParser()
    except rospy.ROSInterruptException:
        pass