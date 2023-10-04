#!/usr/bin/env python3
#-*- coding:utf-8 -*-

from Perception import *
import cv2
import rospy
import sys

from morai_msgs.msg import ObjectStatusList, GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage

class Main:
    def __init__(self):
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.radar_pub = rospy.Publisher('forward_object',ObjectStatusList, queue_size=1)
        self.traffic_light_pub = rospy.Publisher('traffic_data', GetTrafficLightStatus, queue_size=1)
        try:
            self.yolo = YOLO(sys.argv[1])
        except:
            self.yolo=YOLO("yolov5s_openvino_model/")
        self.radar = Radar()
        self.traffic_light = TrafficLight()
        
        self.image = None

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def publish(self):
        self.radar_pub.publish(radar_result)
        self.traffic_light_pub.publish(traffic_light_result)


if __name__ == "__main__":
    if(len(sys.argv)==1):
        print("Don't Input YOLOv5 Model Name")
        print("Default : yolov5s_openvino_model inference:150ms mAP@.5:0.712 mAP50-95:0.472")
    else:
        print(f"Model : {sys.argv[1]}")
    rospy.init_node("perception", anonymous=True)
    main = Main()
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

        if main.image is None : 
            continue 
        if main.radar.is_odom is False:
            continue
        if main.radar.radar_data is None:
            continue
  
        result = main.yolo.get_result(main.image)        
        radar_result = main.radar.get_radar_object_status_list(main.image,result)
        traffic_light_result = main.traffic_light.get_traffic_light_status(main.image, result)

        main.publish()