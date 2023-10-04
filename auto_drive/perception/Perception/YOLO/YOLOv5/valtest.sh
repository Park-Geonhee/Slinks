# python3 val.py --weights yolov5n.pt
# python3 val.py --weights yolov5s.pt
# python3 val.py --weights yolov5s.onnx
python3 val.py --weights yolov5s-int8.tflite
python3 val.py --weights yolov5s-fp16.tflite
