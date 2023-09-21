#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import RadarDetections, RadarDetection, ObjectStatusList, ObjectStatus
from std_msgs.msg import Int32

class Receiver :
    def __init__(self):
        rospy.Subscriber("/from_sender", Int32, self.callback)
        self.data = None
        self.cnt = 0

    def callback(self, msg):
        self.data = msg
        print(msg)

if __name__ =="__main__":
    rospy.init_node("receiver",anonymous=True)
    receiver = Receiver()
    rospy.spin()