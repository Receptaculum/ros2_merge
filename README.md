# ROS2 기반 자율주행 프로그램

## 초기 설정

- colcon build --symlink-install로 빌드 작업을 우선적으로 수행해야 함

- image_publisher.py에서 FRAME_SRC의 경로를 본인의 환경에 맞게 설정해야 정상 동작함

    ex) FRAME_SRC = "/home/[사용자 이름]/ros2_merge/src/camera_perception_pkg/camera_perception_pkg/lib/test_video.mp4"

    (Github로부터 동일한 이름으로 Clone했다는 전제에서, 사용자 이름에 해당하는 부분만 변경하면 동작에 문제가 없을 것임)

## 수정 사항

### ver. 1.0328.1
- image_publishr, yolov8, car_info_extractor, traffic_light_detector, lidar_publisher, lidar_processor, lidar_object_detector 추가

### ver. 1.0329.1
- serial_communicator에서 음수를 전송할 수 있도록 수정

### ver. 1.0330.1
- lidar_processor 기능을 lidar_publisher에 통합

### ver. 1.0330.2
- lidar_debugger 추가

### ver. 1.0401.1
- arduino 코드 추가 (미완성)

### ver. 1.0403.1
- lane_info_extractor 코드 통합
- data_debugger 기능 추가

### ver. 1.0403.2
- CPFL 의존성 제거
- 코드 최적화

### ver. 1.0403.2
- LPFL 의존성 제거

### ver. 1.0404.1
- motion_planner 추가 및 기본 골격 형성

### ver. 1.0406.1
- motion_planner의 lane change state 알고리즘 개선

### ver. 1.0406.2
- UART 수신 데이터를 arduino_data Topic으로 전송하는 기능 추가

### ver. 1.0408.1
- 주행 알고리즘 개선

### ver. 1.0412.1
- driving_algorithm 디렉터리 생성 (각 mission별 motion_planner.py 관리를 위함)

### ver. 1.0412.2
- launch 파일 생성을 위한 execution_pkg 생성

### ver. 1.0412.3
- crosswalk와 traffic_light에 대한 카운트 기능 추가

### ver. 1.0415.1
- mission 2에 대한 motion_planner 추가

### ver. 1.0416.1
- car_info에서 area를 추출할 수 있는 기능 추가

### ver. 1.0416.2
- 차량 차선 존재 여부 판단 기능 개선

### ver. 1.0417.1
- MIDAS 기반 영상 깊이 추정 Node 추가

### ver. 1.0418.1
- 딥러닝 가속 옵션 추가

### ver. 1.0419.1
- lidar_debugger 예외 처리 강화

### ver. 1.0419.2
- 2개의 영역에 대한 LIDAR 감지 기능 추가

### ver. 1.0419.3
- LIDAR 영역 내의 점 개수에 따른 동작 설정 기능 추가

### ver. 1.0422.1
- Mission 1에 대한 개선 작업 수행

### ver. 1.0423.1
- Mission 1에 대한 Fine Tuning 작업 수행

### ver. 1.0425.1
- motion_planner 구조 개선
- depth_extractor 내부 변수 변경

### ver. 1.0425.2
- 후방 카메라를 위한 image_publisher 및 yolov8 추가

### ver. 1.0426.1
- 주차 관련 노드 추가

### ver. 1.0427.1
- 기존 serial_communicator 간 호환성 확보

## 계획
- motion_planner State 추가 및 개선
- Arduino 최적화