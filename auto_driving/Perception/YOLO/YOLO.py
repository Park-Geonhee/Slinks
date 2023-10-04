#!/usr/bin/env python3
import torch
import os

PATH = os.path.dirname(os.path.realpath(__file__)) + '/YOLOv5/'
class YOLO:

    def __init__(self,MODEL):
        self.model = torch.hub.load(PATH, 'custom', MODEL, source='local')

    def get_result(self, image):
        result = self.model(image).pandas().xyxy[0]
        
        return result
    
