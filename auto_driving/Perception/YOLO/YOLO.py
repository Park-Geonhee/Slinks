#!/usr/bin/env python3
import torch
import os

PATH = os.path.dirname(os.path.realpath(__file__))
class YOLO:
    def __init__(self):
        self.model = torch.hub.load(PATH, 'custom', 'yolov5s.onnx', source='local')

    def get_result(self, image):
        result = self.model(image).pandas().xyxy[0] # yolov5 inference
        return result
    
