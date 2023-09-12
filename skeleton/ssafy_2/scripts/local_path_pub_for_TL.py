#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

# 노드 실행 순서 
# 1. Global Path 와 Odometry 데이터 subscriber 생성 
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리 
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("global_path", Path, self.global_path_callback)


        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1)
        #self.local_traffic_lights_pub = rospy.Publisher('local_traffic_lights', Path, queue_size=1)
        
        self.is_odom = False # Odometry 데이터가 수신되었는지 확인하기 위한 flag (수신이 되었을 때 True로 바뀐다)
        self.is_path = False # Global path 데이터가 수신되었는지 확인하기 위한 flag (수신이 되었을 때 True로 바뀐다)

        self.local_path_size = 100  # Can be adjusted based on your specific needs
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
   
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                min_dis = float('inf')
                current_waypoint = -1
                for i, pose in enumerate(self.global_path_msg.poses):
                    distance = sqrt(pow((pose.pose.position.x - x), 2) + pow((pose.pose.position.y - y), 2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i
                
                #TODO: (6) 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리
                if current_waypoint != -1:
                    
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        local_path_msg.poses = self.global_path_msg.poses[current_waypoint:current_waypoint+self.local_path_size]
                    else:
                        local_path_msg.poses = self.global_path_msg.poses[current_waypoint:]

                if local_path_msg.poses:
                    rospy.loginfo("Local path message is not empty.")
                

                print(x,y)
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg        

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
