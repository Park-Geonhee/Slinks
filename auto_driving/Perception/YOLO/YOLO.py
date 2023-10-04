#!/usr/bin/env python3
import torch
import os

PATH = os.path.dirname(os.path.realpath(__file__)) + '/YOLOv5/'
class YOLO:
<<<<<<< HEAD
    def __init__(self,MODEL):
        self.model = torch.hub.load(PATH, 'custom', MODEL, source='local')

    def get_result(self, image):
        result = self.model(image)
        print(result)
        return result.pandas().xyxy[0]
=======
    def __init__(self):
        self.model = torch.hub.load(PATH, 'custom', 'yolov5n.pt', source='local')

    def get_result(self, image):
        result = self.model(image).pandas().xyxy[0]
        
        return result
>>>>>>> ed163e9156c7c24f88d3e41411bb272a1daae5fa
    
