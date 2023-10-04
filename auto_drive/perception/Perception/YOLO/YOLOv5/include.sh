python3 export.py --weights yolov5s.pt --include openvino
python3 export.py --weights yolov5s.pt --include coreml 
python3 export.py --weights yolov5s.pt --include pb
python3 export.py --weights yolov5s.pt --include saved_model
python3 export.py --weights yolov5s.pt --include onnx
python3 export.py --weights yolov5s.pt --include tflite
python3 export.py --weights yolov5s.pt --include tflite --int
