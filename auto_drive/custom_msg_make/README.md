## Custom Message

ROS 통신에서 주고받을 데이터의 형태를 정의할 수 있습니다.

이를 **메시지**라고 합니다.

현재 프로젝트에서는, 기존의 **MORAI msg** 가 제공하는 메세지 타입 뿐만 아니라,

link, node 와 같은 데이터를 정제해야 해서 다음과 같은 메세지 타입을 정의했습니다.


### custom_link_paser.msg
```
string link_idx
float64[] stop_line_point
bool is_on_stop_line
bool[] possible_lattice_pathes
```
## 프로젝트에 메시지 적용하기

메시지 타입을 만들기 위해서는 다음과 같은 절차가 필요로 합니다.

해당 과정은 패키지가 빌드되어있다는 가정 하에 진행합니다.

### 0. 패키지 빌드 (선행 과정)

패키지를 빌드하였을 경우 프로젝트의 폴더 구조는 다음과 같습니다.

```
catkin_ws
└── src
    ├── CMakeLists.txt -> ...
    ├── MORAI-ROS_morai_msgs -> ...
    └── ssafy_ad
        ├── S09P22A701
        └── src
```
해당 디렉토리 구조가 완성되어 있어야 합니다.

### 1. custom_msg 폴더 다운로드

현재 디렉토리의 **msg**, **packge.xml**, **CmakeLists.txt** 파일을 `ssafy_ad` 디렉토리 경로에 위치시킵니다.

이 때 폴더 구조는 다음과 같습니다.
```
catkin_ws
└── src
    ├── CMakeLists.txt -> ...
    ├── MORAI-ROS_morai_msgs -> ...
    └── ssafy_ad
        ├── CMakeLists.txt
        ├── S09P22A701
        ├── msg
        ├── package.xml
        └── src
```

### 2. catkin_make

프로젝트 구조의 `catkin_ws` 폴더로 이동하여 터미널을 킨 뒤, 다음과 같이 입력합니다.

```bash
$catkin_make
```
이렇게 되면 현재 프로젝트 내에서는 `custom_link_parser` 메시지 타입을 사용할 수 있습니다.

### 3. 사용하기

`custom_link_parser` 를 하나의 메시지 타입으로, 구독 혹은 발행을 할 수 있습니다.

다음은 사용 예시입니다.

```python
# Publish
self.link_info_pub = rospy.Publisher("link_info", custom_link_parser, queue_size=1)
self.link_info_pub.publish(link_info_msg)
# Subscribe
 rospy.Subscriber("/link_info", custom_link_parser, self.get_link_info_callback)
```