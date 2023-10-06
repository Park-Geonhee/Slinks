#  자율주행 SLINKS

## 낯선 여행지를 자율 주행으로 편안하게
SLINKS 앱 서비스는 <strong>낯선 여행지</strong>에서도 <strong>다른 유저</strong>의 여행 기록을<br>
참고, <strong>자율 주행</strong>이 가능하게 해주는 <strong>쉽고 간편</strong>한 서비스입니다.

### 메인 기능
- Yolov5 모델을 통한 실시간 신호, 차선 인식<b>안전한 주행</b>
- Astar 알고리즘 기반 경로 추종 <b>신속한 경로 생성</b>
- - -

## 🎬 시연 영상 및 UCC

## 🔑 핵심 기능

### APP


### 자율 주행

- - - 

## 📆 제작 기간 및 인원
제작 기간 : 2023. 08.21 ~ 2023. 10. 06 (7주) <br/>
참여 인원 : 6인

### Moblie
### 👨‍💻 [유지나]
### 자율 주행
### 👨‍💻 [김도훈](https://github.com/bmryu0501) : 지역 경로, Kotlin & ROS Socket 통신, 실시간 위치 표시, UCC 제작
### 👩‍💻 [박건희](https://github.com/201611099) : 차량 제어 및 경로 추종, 속도 계획, 전방 거리 유지, 전역 경로 생성(Astar)
### 👨‍💻 [서강운](https://github.com/Jongwon97) : 전역 경로 생성(Dijkstra, Astar), 회피 경로 생성, V2X(신호등, 정지선) 기반 차량 주행
### 👨‍💻 [이승혁](https://github.com/hun23) : OpenCV, Image Pre-processing, Lane Detect, YOLOv5 Object Detect
### 👩‍💻 [홍의선](https://github.com/twoju) : Sensor Calibration, Sensor Fusion, Object Detect(Radar/Lidar)

- - - 

## 📚 시스템 구성성


### Backend
- Spring Boot
- JPA
- webSocket

### Frontend

### ROS 환경
- docker
- ROS Melodic
- ROS Noetic

### 인지
- OpenCV
- YOLOv5
- RANSAC Regressor
- DBSCAN 
- HOG Discriptor

## 판단
- Dijkstra
- Astar
- V2X

## 제어
- PID Control
- Pure Persuit
- Adaptive Cruise Control

## 🔎 프로젝트 구성

### ⚙ [포팅매뉴얼](./exec/Porting_Manual.pdf)
### 🔗[시스템 모식도](./exec/기능명세서.pdf)
### 🔗[시스템 구조](./exec/Directory_Tree.txt)
### 🖼 [기능명세](./exec/기능명세서.pdf)


