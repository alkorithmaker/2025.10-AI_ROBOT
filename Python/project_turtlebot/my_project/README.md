Tesla Auto Smart Summon Concept를 적용한 ROS 2 기반 자율 주행 로봇 제어 시스템

테슬라의 Auto Smart Summon 기능을 모티브로 한 ROS 2 프로젝트입니다. 
PySide6 GUI 인터페이스를 통해 로봇의 수동 제어와 자율 주행 모드 전환이 가능하며, 
ROS 2 Action Server를 통해 장애물 밀집 지역에서도 안정적으로 목표 좌표까지 도달하는 지능형 소환 기능을 제공합니다.

GUI
Manual Control: 방향키 버튼을 이용한 로봇 원격 조종.
Auto-Smart-Summon: 랜덤 목표 지점 생성 및 자율 주행 시각화 모니터링.
Real-time Status: 서비스(Trigger)를 통한 로봇의 현재 상태(회피 중, 도착 등) 확인.

구성 요소
SummonNode (GUI Client): * PySide6 기반 인터페이스.로봇의 위치를 시각화
SummonServer (Robot Server): Lidar 데이터를 분석하여 장애물 회피 알고리즘 수행. 목적지 도달을 위한 가중치 기반 조향 제어.

토픽 서비스 액션
Topic	/cmd_vel	geometry_msgs/Twist	로봇 이동 제어
Topic	/scan	sensor_msgs/LaserScan	장애물 감지
Topic	/odom	nav_msgs/Odometry	실시간 위치 및 경로 시각화
Action	summon_robot	Summon	자율 주행 및 피드백 전송
Service	get_robot_state	std_srvs/Trigger	로봇 현재 상태 문자열 수신

핵심 로직: 지능형 장애물 회피
기계적인 회피 패턴을 탈피하기 위해 확률적 회피 알고리즘을 적용했습니다.
동적 회피 제어: 정면(40)과 측면(20)의 장애물 거리를 독립적으로 판단.
공간 탐색: 좌/우 센서의 최솟값을 비교하여 더 여유 있는 공간으로 즉시 회전 방향(avoid_dir) 결정.
가변성 부여 (Randomness):회피 회전 속도(1.0~2.0 rad/s)와 탈출 가속도(0.3~0.7 m/s)에 랜덤 값을 적용하여 특정 지형에서 로봇이 갇히는 현상(Deadlock) 방지.
세이프티 타이머: 장애물 감지 구역을 벗어난 후에도 일정 시간(safety_timer) 동안 가속 주행을 유지하여 안정적인 탈출 보장.

설치 방법
1. my_project 패키지 설치
2. my_project 폴더에 qt.ui qt_ui.py summon_gui.py summon_server.py 파일을 생성 
3. create_map에 코드를 cmd에 실행 (/home/robot/robot_ws/src/my_project/worlds/my_obstacle.world)위치에 맵이 생성됨
4. setup.py 설정함
5. 빌드 및 설치

실행 방법
1. 첫 번째 cmd(가제보 월드 실행): ros2 launch gazebo_ros gazebo.launch.py world:=$HOME/robot_ws/src/my_project/worlds/my_obstacle.world
2. 두 번째 cmd(터틀봇 소환): ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0
3. 세 번째 cmd(gui 실행):ros2 run my_project summon_gui
4. 네 번째 cmd(액션 서버 실행):ros2 run my_project summon_server
