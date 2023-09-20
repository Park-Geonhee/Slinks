#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
from nav_msgs.msg import Odometry
from lib.mgeo.class_defs import *
from std_msgs.msg import String

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

class LinkParser:
    def __init__(self):
        rospy.init_node('LinkParser', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.get_link_pub = rospy.Publisher("/get_cur_link", String, queue_size=1)

        # get Mgeo data
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)
        
        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.is_odom = False

        while True:
            if self.is_odom == True : break
            else : rospy.loginfo("Waiting odometry data")

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            # 1. find nearest node
            near_nodes = self.find_near_5_nodes()

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
            print(current_link.idx)
            self.get_link_pub.publish(current_link.idx)

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
    def find_near_5_nodes(self):
        result = [[21e8, Node()], [21e8, Node()], [21e8, Node()], [21e8, Node()], [21e8, Node()]]
        
        for node_idx, node in self.nodes.items():
            if (self.x - node.point[0]>100) or (self.y-node.point[1])>100 : continue
            ddist = pow(self.x - node.point[0],2) + pow(self.y - node.point[1],2)
            if ddist < result[4][0]:
                result[4] = [ddist, node]
                result.sort(key=lambda x:x[0])
        return result
            
if __name__ == '__main__':
    try:
        Link_Parser = LinkParser()
    except rospy.ROSInterruptException:
        pass