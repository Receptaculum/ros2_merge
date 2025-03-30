import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import logging


## <Parameter> #######################################################################################

# 구독 토픽 이름
SUB_TOPIC_NAME = 'lidar_processed' 

# 발행 토픽 이름
PUB_TOPIC_NAME = 'lidar_data'

# 로깅 여부
LOG = True

# 감지 카운트
COUNT = 3

# 각도 설정
START_ANGLE = 0  # 감지 각도 범위의 시작 값
END_ANGLE = 30   # 감지 각도 범위의 끝 값
        
# 범위 설정
RANGE_MIN = 0.1  # 감지 거리 범위의 최소값 [m]
RANGE_MAX = 1.0  # 감지 거리 범위의 최대값 [m]

######################################################################################################


## <LIDAR> ###########################################################################################
#        270
#       #######  (Motor)
#     0 # 본체 ######### 180     (Counter-Clockwise)
#       #######
#         90
######################################################################################################


class lidar_object_detector(Node):
    def __init__(self):
        super().__init__('lidar_object_detector')

        # QOS 선언
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publisher / Subscriber 선언
        self.subscriber = self.create_subscription(LaserScan, SUB_TOPIC_NAME, self.lidar_callback, self.qos_profile)
        self.publisher = self.create_publisher(Bool, PUB_TOPIC_NAME, self.qos_profile) 

        # 감지 카운트를 위한 저장소
        self.detection_reg = []
        self.state_reg = bool()

        # 로깅 여부 설정
        if LOG == False: 
            self.get_logger().set_level(logging.FATAL)


    def lidar_callback(self, msg):
        # 수신받은 거리 데이터 추출
        ranges = msg.ranges

        # 감지 여부 추출
        detected = self.detect_object(ranges=ranges, start_angle=START_ANGLE, end_angle=END_ANGLE, range_min=RANGE_MIN, range_max=RANGE_MAX)
        
        # 감지 카운트
        detection_result = self.check_consecutive_detections(detected, COUNT)

        # 메시지 생성 및 전송
        detection_msg = Bool()
        detection_msg.data = detection_result
        self.publisher.publish(detection_msg)

        # 로깅 데이터 기록
        self.get_logger().info(f'Detection Result = {detection_result}')


    def detect_object(self, ranges, start_angle, end_angle, range_min, range_max):
        # 항상 360 출력
        total_angle = len(ranges)
    
        # 0 ~ 360 범위로 고정
        if start_angle > end_angle:
            end_angle += total_angle
        
        # 설정 구간 내에 값 존재 여부 확인
        for i in range(start_angle, end_angle + 1):
            i = i % total_angle
        
            if range_min <= ranges[i] <= range_max:
                return True
        
        # 감지되지 않았을 경우 반환값
        return False
    

    def check_consecutive_detections(self, detection, cnt):
        # 레지스터에 감지 데이터 추가
        self.detection_reg.append(detection)

        # 레지스터 저장 개수 제한
        if len(self.detection_reg) > cnt:
            self.detection_reg.pop(0)

        # T -> F 변환 조건
        if self.state_reg and self.detection_reg.count(False) >= cnt:
            self.state_reg = False
        
        # F -> T 변환 조건
        elif not self.state_reg and self.detection_reg.count(True) >= cnt:
            self.state_reg = True

        return self.state_reg


def main(args=None):
    rclpy.init(args=args)
    object_detector_node = lidar_object_detector()

    rclpy.spin(object_detector_node)
    
    object_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()