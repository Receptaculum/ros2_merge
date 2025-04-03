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


## 계획

- motion_planner 추가
- lane_info_extractor 추가
- Arduino 최적화