#!/usr/bin/env python3
import torch
import os

PATH = os.path.dirname(os.path.realpath(__file__)) + '/YOLOv5/'
class YOLO:
<<<<<<< HEAD
    def __init__(self):
        self.model = torch.hub.load(PATH, 'custom', 'yolov5s.onnx', source='local', device=1)
=======
    def __init__(self,MODEL):
        self.model = torch.hub.load(PATH, 'custom', MODEL, source='local')
>>>>>>> a95aaf4a1ca06165539e7cf0e151b6c1b6db94a5

    def get_result(self, image):
        result = self.model(image).pandas().xyxy[0]
        
        return result
    
