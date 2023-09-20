#! /usr/bin/env python3
#-*- coding:utf-8 -*-
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridgeError
from sensor_msgs.msg import CompressedImage

# Constants.
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.45
CONFIDENCE_THRESHOLD = 0.45
 
# Text parameters.
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
THICKNESS = 1
 
# Colors.
BLACK  = (0,0,0)
BLUE   = (255,178,50)
RED = (0,0,255 )

class TEST:
      def __init__(self):
            # Load class names.
            self.classesFile = "/home/leesh/catkin_ws/src/ssafy_ad/S09P22A701/seunghyuk/temp/coco.names"
            self.classes = None
            self.img=None
            rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
            with open(self.classesFile, 'rt') as f:
                  self.classes = f.read().rstrip('\n').split('\n')

            modelWeights = "/home/leesh/catkin_ws/src/ssafy_ad/S09P22A701/seunghyuk/temp/yolov5s.onnx"
            net = cv2.dnn.readNet(modelWeights)
            rate = rospy.Rate(5)

            while not rospy.is_shutdown():
                  if self.img is not None:
                        # Process image.
                        detections = self.pre_process(self.img, net)
                        self.image = self.post_process(self.img.copy(), detections)
                        t, _ = net.getPerfProfile()

                        #label = 'Inference time: %.2f ms' % (t * 1000.0 /  cv2.getTickFrequency())

                        #cv2.putText(self.image, label, (20, 40), FONT_FACE, FONT_SCALE,  (0, 0, 255), THICKNESS, cv2.LINE_AA)
                        # cv2.imshow('Output', self.image)
                        # cv2.waitKey(1)
                  rate.sleep()

      def draw_label(self,im, label, x, y):
            text_size = cv2.getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS)
            dim, baseline = text_size[0], text_size[1]
            cv2.putText(im, label, (x, y + dim[1]), FONT_FACE, FONT_SCALE, RED, THICKNESS, cv2.LINE_AA)

      def pre_process(self,input_image, net):
            blob = cv2.dnn.blobFromImage(input_image, 1/255,  (INPUT_WIDTH, INPUT_HEIGHT), [0,0,0], 1, crop=False)
            net.setInput(blob)
            outputs = net.forward(net.getUnconnectedOutLayersNames())
            return outputs

      def post_process(self,input_image, outputs):
            class_ids = []
            confidences = []
            traffic_lights=None
            traffic_lights_conf=-1
            boxes = []
            rows = outputs[0].shape[1]
            image_height, image_width = input_image.shape[:2]
            x_factor = image_width / INPUT_WIDTH
            y_factor =  image_height / INPUT_HEIGHT

            for r in range(rows):
                  row = outputs[0][0][r]
                  confidence = row[4]
                  if confidence >= CONFIDENCE_THRESHOLD:
                        classes_scores = row[5:]
                        class_id = np.argmax(classes_scores)
                        if (classes_scores[class_id] > SCORE_THRESHOLD):
                              cx, cy, w, h = row[0], row[1], row[2], row[3]
                              left = int((cx - w/2) * x_factor)
                              top = int((cy - h/2) * y_factor)
                              width = int(w * x_factor)
                              height = int(h * y_factor)
                              box = np.array([left, top, width, height])
                              confidences.append(confidence)
                              class_ids.append(class_id)
                              boxes.append(box)

            indices = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)

            for i in indices:
                  box = boxes[i]
                  left = box[0]
                  top = box[1]
                  width = box[2]
                  height = box[3]             
                  if(class_ids[i]==9):
                        if(height > width):
                              continue
                        try:
                              if(traffic_lights==None or width*height > traffic_lights[2]*traffic_lights[3]):
                                    traffic_lights, traffic_lights_conf=box , confidences[i]
                        except:
                              pass
                  else:
                        cv2.rectangle(input_image, (left, top), (left + width, top + height), RED, THICKNESS)
                        label = "{}:{:.2f}".format(self.classes[class_ids[i]], confidences[i])             
                        self.draw_label(input_image, label, left, top)
            
            if(traffic_lights is not None):
                  left,top,width,height = traffic_lights
                  cv2.rectangle(input_image, (left, top), (left + width, top + height), RED, THICKNESS)
                  label = "{}:{:.2f}".format("traffic light", traffic_lights_conf)             
                  self.draw_label(input_image, label, left, top)
                  print("Find TRAFICLIGHT!!!")
            return input_image                        

      def callback(self,msg):
            arr= np.frombuffer(msg.data,np.uint8)
            self.img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
      
if __name__ == '__main__':
      try:
            rospy.init_node('lane_fitting', anonymous=True)
            t = TEST()
      except rospy.ROSInterruptException:
            pass
