## roslaunch

ROS 노드 파일들을 한번에 `rosrun`(실행) 하기 위한 파일입니다.

MORAI Simulator를 실행한 후 터미널에 다음과 같이 입력하면 실행할 수 있습니다.

```bash
$source ~/catkin_ws/devel/setup.bash
$roslaunch ssafy_ad example.launch
```
`example.launch` 파일에 사용할 launch 파일을 입력합니다.

## .launch

현 디렉토리의 launch 파일은 다음과 같습니다.

`sangam_dijkstra.launch` : rviz로 지점 입력, dijkstra 알고리즘으로 전역 경로 생성 

`sangam_astar.launch` : rviz로 지점 입력, astar 알고리즘으로 전역 경로 생성

`sangam_web.launch` : mobile에서 지점 입력, astar 알고리즘으로 전역 경로 생성

`sangan_final.launch` : `sangam_web.launch` 에서 센서 데이터 습득 기능 추가