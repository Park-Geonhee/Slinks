import rospy
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd,GetTrafficLightStatus, TrafficLight

'''
Ver_1
    1. GetTrafficLightStatus를 sub하여 현재 도로의 신호등 정보를 가져온다.
    2. 정지신호라면 신호등 위치(point)에 맞춰서 속도를 0으로 맞춘다.
    3. 정지신호가 아니라면 acc가 그대로 작동한다.


Ver_2
1. global path의 링크 순서를 sub한다
2. mgeo_data의 traffic_light_set을 가져오고, 링크 순서에 맞게 신호등을 queue 에 담는다.
3. GetTrafficLightStatus의 

'''


class traffic_light:
    def __init__(self):
        rospy.init_node('traffic_light', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("global_path", Path, self.global_path_callback)
        #rospy.Subscriber("local_traffic_lights", list, self.local_traffic_lights_callback)
        rospy.Subscriber("traffic_light_info", GetTrafficLightStatus, self.traffic_light_callback)

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg        

    def local_traffic_lights_callback(self,msg):
        self.local_traffic_lights = msg

    def traffic_light_callback(self,msg):
        self.traffic_light_info = msg



if __name__ == '__main__':
    try:
        test_track=traffic_light()
    except rospy.ROSInterruptException:
        pass
