## <img src="ssafy_app/googlemap/app/src/main/res/drawable/logo_yellow.png" height="24"> 자율주행

## 낯선 여행지를 자율 주행으로 편안하게
SLINKS 앱 서비스는 <strong>낯선 여행지</strong>에서도 <strong>다른 유저</strong>의 여행 기록을<br>
참고, <strong>자율 주행</strong>이 가능하게 해주는 <strong>쉽고 간편</strong>한 서비스입니다.

- - -

## 🎬 시연 영상 및 UCC
[UCC](https://drive.google.com/file/d/1JkjbP2QPGlyhCTZMykJQ7qb9ioyMHimS/view?usp=sharing)
[시연영상](https://drive.google.com/file/d/1ykUax0QnmBkUvWOW1vizO9ojfq1g2f5N/view?usp=sharing)

## 🔑 핵심 기능

### APP
- AI 일기 생성
- 지도에서 일기가 생성된 장소 확인
- 여행계획 생성 및 관리


### 자율 주행
- - - 
- Yolov5 모델을 통한 실시간 신호, 차선 인식<b>안전한 주행</b>
- Astar 알고리즘 기반 경로 추종 <b>신속한 경로 생성</b>


## 📆 제작 기간 및 인원
제작 기간 : 2023. 08.21 ~ 2023. 10. 06 (7주) <br/>
참여 인원 : 6인

### Moblie
### 👩‍💻 유지나 : 기획, 디자인, 서버개발, 모바일개발, 발표
### 자율 주행
### 👨‍💻 [김도훈](https://github.com/donny0331) : 지역 경로, Kotlin & ROS Socket 통신, 실시간 위치 표시, UCC 제작
### 👨‍💻 [박건희](https://github.com/geon4415) : 차량 제어 및 경로 추종, 속도 계획, 전방 거리 유지, 전역 경로 생성(Astar)
### 👨‍💻 [서강운](https://github.com/sku379829) : 전역 경로 생성(Dijkstra, Astar), 회피 경로 생성, V2X(신호등, 정지선) 기반 차량 주행
### 👨‍💻 [이승혁](https://github.com/leeseunghyuk0228) : OpenCV, Image Pre-processing, Lane Detect, YOLOv5 Object Detect
### 👨‍💻 [홍의선](https://github.com/hon3538) : Sensor Calibration, Sensor Fusion, Object Detect(Radar/Lidar)

- - - 

## 📚 시스템 구성성


### Backend
- Spring Boot
- JPA
- MariaDB
- webSocket

### Moblie
- Kotlin

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
### 🔗[시스템 모식도](./exec/시스템모식도.png)
### 🔗[시스템 구조](./exec/Directory_Tree.txt)
### 🖼 [기능명세](./exec/기능명세서.pdf)
### 🖼 [기획안](./exec/기획안.pdf)
### 🖼 [ER Diagram](./exec/ERD.png)


