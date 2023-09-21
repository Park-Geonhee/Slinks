import tensorflow as tf
import numpy as np
import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in',default='/')
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',default='yolov5-int8.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',default='coco.names')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support thentered, errors may occur.',default='640x640')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

interpreter = tf.lite.Interpreter(model_path="/home/leesh/catkin_ws/src/ssafy_ad/S09P22A701/seunghyuk/traffic_light/yolov5s-int8.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]['shape']
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

# print("## input detail##")
# print(input_details)

# print("## ouput detail ##")
# print(output_details)

# print(height, width)


# floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

outname = output_details[0]['name']
boxes_idx, classes_idx, scores_idx = 1, 3, 0
frame = cv2.imread('/home/leesh/catkin_ws/src/ssafy_ad/S09P22A701/seunghyuk/traffic_light/image.jpg')
frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
frame_resized = cv2.resize(frame_rgb, (width, height))
print(frame_resized.shape)
input_data = np.expand_dims(frame_resized, axis=0)
# input_data = (np.float32(input_data) - input_mean) / input_std

interpreter.set_tensor(input_details[0]['index'],input_data)
interpreter.invoke()
output_data = interpreter.get_tensor(output_details[0]['index'])[0]
print(output_data[-1])
# print(interpreter.et_tensor(output_details))
# boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])
# classes = interpreter.get_tensor(output_details[classes_idx]['index'][0])
# scores = interpreter.get_tensor(output_details[scores_idx]['index'][0])

# print(boxes)
# print(classes)
# print(scores)