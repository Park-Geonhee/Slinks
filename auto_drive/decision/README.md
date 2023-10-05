## Decision & Control

해당 디렉토리는 자율주행의 판단 및 제어를 담당하는 파트입니다.

판단, 제어에서의 분류 및 각 파일 별 기능은 다음과 같습니다.


### 데이터 습득

`sangam_mgeo_puh.py` : 맵의 link, node 데이터를 발행합니다.

`sangam_gpsimu_parser.py` : 차량의 GPS, IMU 센서값을 습득하고 전처리합니다.

`sangam_link_parser.py` : 맵의 link, node, 정지선 정보 등을 전처리합니다.

`sangam_get_point.py` : Web에서 출발지, 경유지, 도착지 정보를 습득합니다.

### 전역 경로 계획

`sangam_mgeo_global_dijkstra` : dijksta 알고리즘을 통해 전역 경로를 생성합니다.

`sangam_mgeo_global_astar` : astar 알고리즘을 통해 전역 경로를 생성합니다.

`*.rivz.py` : rviz 프로그램을 통해 목표지점을 설정한 경우 사용합니다.

`*web.py` : 서비스의 mobile 단에서 목표지점을 설정한 경우 사용합니다.

### 지역 경로 계획

`sangam_local_path_pub.py` : 생성된 전역경로를 기반으로 차량이 추종할 수 있게 지역 경로로 처리하여 발행합니다.

`sangam_lattice_planner.py` : 지역 경로에 장애물 정보를 추가로 고려하여 지역 경로 수준의 회피 경로를 발행합니다.

### 경로 추종, 차량 제어

`sangam_acc_with_tl.py` : 발행된 지역 경로를 추종하며 차량의 종방향 제어, 횡방향 제어를 수행합니다.

차량 전방의 움직이는 물체와 지역 경로의 곡률 등을 고려하여 종방향 제어의 값을 추가로 조절합니다.