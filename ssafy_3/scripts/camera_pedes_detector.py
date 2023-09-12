#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

def non_maximum_supression(bboxes, threshold=0.3):
    
    bboxes = sorted(bboxes, key=lambda detections: detections[3],reverse=True)

    new_bboxes=[]
    new_bboxes.append(bboxes[0])
    
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):
        x1_tl, y1_tl = bbox[0],  bbox[1] 
        x1_br, y1_br = bbox[0] + bbox[2], bbox[1] + bbox[3] 
        for new_bbox in new_bboxes:
            x2_tl, y2_tl = new_bbox[0],  new_bbox[1]
            x2_br, y2_br = new_bbox[0] + new_bbox[2], new_bbox[1] + new_bbox[3]

            overlap_x=[max(x1_tl,x2_tl), min(x1_br,x2_br)]
            overlap_y=[max(y1_tl,y2_tl), min(y1_br,y2_br)]

            overlap_w = overlap_x[1]-overlap_x[0]
            overlap_h = overlap_y[1]-overlap_y[0]

            overlap_area = max(0,overlap_w) * max(0,overlap_h)+1
            
            area_1 = bbox[2]*bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]

            total_area = area_1 + area_2 - overlap_area

            overlap_area /= total_area

            if overlap_area < threshold:
                new_bboxes.append(bbox)

    return new_bboxes

class PEDESDetector:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)
        self.rate = rospy.Rate(20)

        self.pedes_detector = cv2.HOGDescriptor()   
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def callback(self, msg):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 0, 255)
        font_thickness = 2
        self.rate.sleep()
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_gray, winStride=(4,4), padding=(2,2), scale=32)
        
        if len(rects_temp) != 0:
            rects = non_maximum_supression(rects_temp)
            for (x,y,w,h) in rects:
                text="pedes"
                cv2.rectangle(img_bgr, (x,y),(x+w,y+h),(0,0,255), 2)
                position=(x-20,y)
                cv2.putText(img_bgr, text, position, font, font_scale, font_color, font_thickness)
                
        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1) 


if __name__ == '__main__':

    rospy.init_node('pedes_detector', anonymous=True)

    pedes_detector = PEDESDetector()

    rospy.spin() 
