#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import RadarDetections, RadarDetection, ObjectStatusList, ObjectStatus
from std_msgs.msg import Int32

class Sender :
    def __init__(self):
        rospy.Subscriber("/radar",RadarDetections,self.callback)
        self.to_receiver = rospy.Publisher("/from_sender", Int32, queue_size=1)
        self.data = None
        self.cnt = Int32()
        self.cnt = 0
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() :
            self.to_receiver.publish(self.cnt)
            print(self.cnt)
            self.cnt = self.cnt + 1
            rate.sleep()

    def callback(self, msg):
        self.data = msg

if __name__ =="__main__":
    rospy.init_node("sender",anonymous=True)
    sender = Sender()
    rospy.spin()