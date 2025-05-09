########################
# For Mission 3 (Real) #
########################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces_pkg.msg import CarData, LaneData, SegmentGroup, MotionCommand, BoolMultiArray
from std_msgs.msg import String, Bool, Int8MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv_bridge
import time

## <Parameter> #####################################################################################

# 구독 토픽 이름
SUB_TOPIC_CAR = "car_data" 
SUB_TOPIC_CAR_REAR = "car_data_rear" 
SUB_TOPIC_LIDAR = "lidar_data"
SUB_TOPIC_YOLO = "segmented_data"
SUB_TOPIC_YOLO_REAR = "segmented_data_rear"
SUB_TOPIC_LINE = "line_data_rear"
SUB_TOPIC_DEPTH = "depth_data"

# 발행 토픽 이름
PUB_TOPIC_NAME = "command_data"

# 연산 주기 설정
PERIOD = 0.1

# 차량 범퍼 위치
BUMPER_POSITION = [320, 462]

# 보정 상수
K_Stanley_1 = 0.38
K_Angle_1 = 0.44

K_Stanley_2 = 0.38
K_Angle_2 = 0.44

# 기준선
BASE_LINE = 320

# 디버그 모드
DEBUG = False

######################################################################################################

class motion_planner(Node):
    def __init__(self):
        super().__init__("motion_planner")

        self.qos_sub = QoSProfile( # Subscriber QOS 설정
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )
        
        self.qos_pub = QoSProfile( # Publisher QOS 설정
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )
        
        # Subsciption 선언
        self.sub_car = self.create_subscription(CarData, SUB_TOPIC_CAR, self.update_car_data, self.qos_sub)
        self.sub_car_rear = self.create_subscription(CarData, SUB_TOPIC_CAR_REAR, self.update_car_rear_data, self.qos_sub)
        self.sub_lidar = self.create_subscription(BoolMultiArray, SUB_TOPIC_LIDAR, self.update_lidar_data, self.qos_sub)
        self.sub_yolo = self.create_subscription(SegmentGroup, SUB_TOPIC_YOLO, self.update_yolo_data, self.qos_sub)
        self.sub_yolo_rear = self.create_subscription(SegmentGroup, SUB_TOPIC_YOLO_REAR, self.update_yolo_rear_data, self.qos_sub)
        self.sub_line = self.create_subscription(Float32MultiArray, SUB_TOPIC_LINE, self.update_line_data, self.qos_sub)
        self.sub_depth = self.create_subscription(Image, SUB_TOPIC_DEPTH, self.update_depth_data, self.qos_sub)

        # Publisher 선언
        self.command_publisher = self.create_publisher(MotionCommand, PUB_TOPIC_NAME, self.qos_pub)

        # CV Bridge Object 선언    
        self.bridge = cv_bridge.CvBridge()

        # 데이터 저장 레지스터 선언
        self.car_data = None
        self.car_rear_data = None
        self.lidar_data = None
        self.yolo_data = None
        self.yolo_rear_data = None
        self.line_data = None
        self.depth_data = None

        # State 저장 레지스터 선언 (0, 1, 2, 3) | 0은 초기화 상태를 의미함
        self.state = 0

        # Timer 선언
        self.timer = self.create_timer(PERIOD, self.motion_decision_callback)

        # 전송 데이터 기억
        self.steer_angle_reg = 0 # send_command 함수에서 사용

##### <변수 업데이트를 위한 함수 선언> #####################################################################

    def update_car_data(self, msg):
        self.car_data = msg # car position        

#######################################################################

    def update_car_rear_data(self, msg):
        self.car_rear_data = msg # car position        

#######################################################################

    def update_lidar_data(self, msg):
        self.lidar_data = msg # T/F

#######################################################################

    def update_yolo_data(self, msg):
        self.yolo_data = msg # segmentation data

#######################################################################

    def update_yolo_rear_data(self, msg):
        self.yolo_rear_data = msg # segmentation data

#######################################################################

    def update_line_data(self, msg):
        self.line_data = msg # line data

#######################################################################

    def update_depth_data(self, msg):
        self.depth_data = msg # Depth Image

######################################################################################################


    # 제어 명령 전송 함수 (-128 ~ 127)
    def send_command(self, steer_angle:int, left_speed:int, right_speed:int):
        msg = MotionCommand()

        # 조향각 데이터가 비어있는 경우
        if steer_angle == None:
            # 이전 조향각 반영
            steer_angle = self.steer_angle_reg

        else:
            # 조향각 업데이트
            self.steer_angle_reg = steer_angle

        msg.steering = steer_angle
        msg.left_speed = left_speed
        msg.right_speed = right_speed

        if DEBUG == True:
            msg = MotionCommand()

        self.command_publisher.publish(msg)


    # Stanley Method 기반 조향각 계산 함수
    def calculate_steering_angle(self, target_point:list, car_center_point:list, path_slope:float, vehicle_speed:int, k_angle, k_stanley):
            # Heading Error
            heading_error = path_slope * k_angle

            # 횡방향 오차 계산
            lateral_error = target_point[0] - car_center_point[0]

            # 조향각 계산
            steering_angle = heading_error + np.arctan(k_stanley * lateral_error / (vehicle_speed + 1e-6))*(180/np.pi)
            return int(np.clip(steering_angle, -40, 40)) # 각도 제한 (-40~40)


    # 판단 로직 작성부
    def motion_decision_callback(self):
        try:
            # State 0 : init_mode
            if self.state == 0:
                self.state = self.init_mode()

            # State 1 : search_mode
            elif self.state == 1:
                self.state = self.search_mode()

            # State 2 : turn_mode
            elif self.state == 2:
                self.state = self.turn_mode()

            # State 3 : stop_mode_1
            elif self.state == 3:
                self.state = self.stop_mode_1()

            # State 4 : back_up_mode
            elif self.state == 4:
                self.state = self.back_up_mode()

            # State 5 : stop_mode_2
            elif self.state == 5:
                self.state = self.stop_mode_2()

            # State 6 : forward_mode
            elif self.state == 6:
                self.state = self.forward_mode()

        except Exception as e:
            self.get_logger().warn(f"{e}")


### <State 정의 함수> ####################################################################

    # State 0
    def init_mode(self) -> int:      
        # 데이터가 전부 수신되었을 경우, 처리 시작 (1 : 전체 확인 | 2 : LIDAR 제외 | 3 : DEPTH 제외)

        #if self.car_data != None and self.lane_data != None and self.traffic_data != None and self.lidar_data != None and self.yolo_data != None and self.depth_data != None:
        #if self.car_data != None and self.lane_data != None and self.traffic_data != None and self.yolo_data != None and self.depth_data != None:
        if self.car_data != None and self.car_rear_data != None and self.lidar_data != None and self.yolo_data != None and self.yolo_rear_data != None and self.line_data != None:

            # 0의 지령값 설정
            self.send_command(steer_angle = 0, left_speed = 0, right_speed = 0)

            # 주행 모드로 반환
            return 1
        

        # 데이터가 전부 수신되지 않았을 경우, 오류 전송
        else:
            self.get_logger().warn("data is not yet accepted")
            self.get_logger().warn(f"{self.car_data != None}, {self.car_rear_data != None}, {self.lidar_data != None}, {self.yolo_data != None}, {self.yolo_rear_data != None}, {self.line_data != None}")
            return 0

########################################################################################

    # State 1
    def search_mode(self) -> int:
        self.get_logger().info(f"search_mode")   

        # 정속 주행
        self.send_command(steer_angle = 0, left_speed = 100, right_speed = 100)
        
        # 우측 LIDAR에 사물이 감지된 경우
        if self.lidar_data.data[1] == True:
            #
            return 2


        # 현 상태 유지
        return 1

########################################################################################

    # State 2
    def turn_mode(self) -> int:
        self.get_logger().info(f"turn_mode")   

        # 좌측 조향 운전
        self.send_command(steer_angle = -30, left_speed = 100, right_speed = 70)

        # 2개의 차량이 시야에 감지된 경우
        if len(self.car_rear_data.x) == 2:
            return 3
        
        # 현 상태 유지
        return 2
    
########################################################################################

    # State 3
    def stop_mode_1(self) -> int:
        self.get_logger().info(f"stop_mode_1")   

        # 정지
        self.send_command(steer_angle = -30, left_speed = 0, right_speed = 0)

        # 1초 지연
        time.sleep(1)

        # 후진 단계로 이동
        return 4

########################################################################################

    # State 4
    def back_up_mode(self) -> int:
        self.get_logger().info(f"back_up_mode")
        car_center_point = [640/2, 480]   

        # 2개의 차량이 감지된 경우
        if len(self.car_rear_data.x) == 2:
            target_point = [sum(self.car_rear_data.x)/2, sum(self.car_rear_data.y)/2]

        # 1개의 차량이 감지된 경우
        elif len(self.car_rear_data.x) == 1:
            # 후방 좌측 차량만 감지될 경우
            if self.car_rear_data.x[0] < 640/2:
                target_point = [(self.car_rear_data.x[0] + 640)/2, None]

            # 후방 우측 차량만 감지될 경우
            elif self.car_rear_data.x[0] > 640/2:
                target_point = [(self.car_rear_data.x[0] + 0)/2, None]

        # 차량이 감지되지 않은 경우
        else:
            target_point = [640/2, None]

        # 조향각 계산
        angle = self.calculate_steering_angle(target_point, car_center_point, 0, 120, 0, 2)

        # 후진 진행
        self.send_command(steer_angle = angle, left_speed = -120, right_speed = -120)

        # LIDAR 양쪽에 장애물 감지시 정지
        if self.lidar_data.data[0] == True and self.lidar_data.data[1] == True:
            return 5

        # 현 상태 유지
        return 4

########################################################################################

    # State 5
    def stop_mode_2(self) -> int:
        self.get_logger().info(f"stop_mode_2")   

        # 정지
        self.send_command(steer_angle = 0, left_speed = 0, right_speed = 0)

        # 1초 지연
        time.sleep(2)

        # 마무리 단계로 이동
        return 6

########################################################################################

    # State 6
    def forward_mode(self) -> int:
        self.get_logger().info(f"forward_mode")   

        # 직진
        self.send_command(steer_angle = 0, left_speed = 200, right_speed = 200)

        # 5초 지연
        time.sleep(2)

        # 우회전
        self.send_command(steer_angle = 30, left_speed = 200, right_speed = 200)

        # 5초 지연
        time.sleep(5.5)

        # 직진
        self.send_command(steer_angle = 0, left_speed = 200, right_speed = 200)

        # Trapping
        while True:
            continue

########################################################################################

def main():
    rclpy.init()
    motion_planner_node = motion_planner()
    rclpy.spin(motion_planner_node)

    motion_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
