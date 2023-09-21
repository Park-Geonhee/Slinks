#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from lib.mgeo.class_defs import *
from std_msgs.msg import String, Bool

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

class LinkParser:
    def __init__(self):
        rospy.init_node('LinkParser', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.get_link_pub = rospy.Publisher("/current_link", String, queue_size=1)
        self.stop_line_pub = rospy.Publisher("/stop_line", Point, queue_size=1)
        self.is_on_stop_line_pub = rospy.Publisher("/on_stop_line", Bool, queue_size=1)

        # get Mgeo data
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PR_Sangam_NoBuildings'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)
        
        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.is_odom = False

        while True:
            if self.is_odom == True : break
            else : rospy.loginfo("Waiting odometry data")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            current_link = self.find_current_link()
            self.find_stop_line(current_link)

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
    def find_near_3_nodes(self):
        result = [[21e8, Node()], [21e8, Node()], [21e8, Node()]]
        
        for node_idx, node in self.nodes.items():
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
            #connected_links = nearest_node.to_links + nearest_node.from_links

        # 3. find nearest link with current position
        current_link = Link()
        min_ddist = 21e8
        for link in connected_links:
            for point in link.points:
                ddist = pow(self.x - point[0],2) + pow(self.y - point[1],2)
                if ddist < min_ddist:
                    min_ddist = ddist
                    current_link = link

        self.get_link_pub.publish(current_link.idx)
        return current_link

    def find_stop_line(self, current_link):
            current_to_node = current_link.get_to_node()
            stop_line_point = Point()
            stop_line_point.x = current_to_node.point[0]
            stop_line_point.y = current_to_node.point[1]
            stop_line_point.z = current_to_node.point[2]

            is_on_stop_line = current_to_node.on_stop_line
            if is_on_stop_line == False:
                next_to_link = current_to_node.to_links
                if (len(next_to_link) == 1) and (next_to_link[0].get_to_node().on_stop_line):
                    next_to_node = next_to_link[0].get_to_node()
                    is_on_stop_line = next_to_node.on_stop_line
                    stop_line_point.x = next_to_node.point[0]
                    stop_line_point.y = next_to_node.point[1]
                    stop_line_point.z = next_to_node.point[2]
                else:
                    pass

            self.stop_line_pub.publish(stop_line_point)
            self.is_on_stop_line_pub.publish(is_on_stop_line)
    


if __name__ == '__main__':
    try:
        Link_Parser = LinkParser()
    except rospy.ROSInterruptException:
        pass