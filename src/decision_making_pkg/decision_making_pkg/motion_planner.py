import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import CarData, LaneData
from std_msgs.msg import String, Bool, Int8MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

## <Parameter> #####################################################################################

# 구독 토픽 이름
SUB_TOPIC_CAR = "car_data" 
SUB_TOPIC_LANE = "lane_data"
SUB_TOPIC_TRAFFIC = "traffic_data"
SUB_TOPIC_LIDAR = "lidar_data"

# 발행 토픽 이름
PUB_TOPIC_NAME = "command_data"

# 연산 주기 설정
PERIOD = 0.1

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
        self.sub_lane = self.create_subscription(LaneData, SUB_TOPIC_LANE, self.update_lane_data, self.qos_sub)
        self.sub_traffic = self.create_subscription(String, SUB_TOPIC_TRAFFIC, self.update_traffic_data, self.qos_sub)
        self.sub_lidar = self.create_subscription(Bool, SUB_TOPIC_LIDAR, self.update_lidar_data, self.qos_sub)

        # Publisher 선언
        self.command_publisher = self.create_publisher(Int8MultiArray, PUB_TOPIC_NAME, self.qos_pub)

        # 데이터 저장 레지스터 선언
        self.car_data = None
        self.lane_data = None
        self.traffic_data = None
        self.lidar_data = None

        # State 저장 레지스터 선언 (1, 2, 3)
        self.state = 1

        # Lane 위치 저장 레지스터 선언 (1, 2)
        self.lane_state = 2

        # Timer 선언
        self.timer = self.create_timer(PERIOD, self.motion_decision_callback) 


    # 변수 업데이트를 위한 함수 선언
    def update_car_data(self, msg):
        self.car_data = msg

    def update_lane_data(self, msg):
        self.lane_data = msg
   
    def update_traffic_data(self, msg):
        self.traffic_data = msg
   
    def update_lidar_data(self, msg):
        self.lidar_data = msg


    # 제어 명령 전송 함수 (-128 ~ 127)
    def send_command(self, steer_angle:int, left_speed:int, right_speed:int):
        msg = Int8MultiArray()
        msg.data = [steer_angle, left_speed, right_speed]

        self.command_publisher.publish(msg)


    # 판단 로직 작성부
    def motion_decision_callback(self):
        # State 1 : drive_mode
        if self.state == 1:
            self.state = self.drive_mode()

        # State 2 : lane_change_mode
        elif self.state == 2:
            self.state = self.lane_change_mode()

        # State 3 : stop_mode
        elif self.state == 3:
            self.state = self.stop_mode()


### <State 정의 함수> ####################################################################

    # State 1
    def drive_mode(self) -> int:
        print("drive_mode")
        self.send_command(steer_angle = 0, left_speed = 1, right_speed = 0)
        return 2 # 다음 상태값 리턴

    # State 2
    def lane_change_mode(self) -> int:
        print("lane_change_mode")
        self.send_command(steer_angle = 125, left_speed = 1, right_speed = 34)
        return 3 # 다음 상태값 리턴

    # State 3
    def stop_mode(self) -> int:
        print("stop_mode")
        self.send_command(steer_angle = 0, left_speed = 0, right_speed = 0)
        return 1 # 다음 상태값 리턴

########################################################################################

def main():
    rclpy.init()
    motion_planner_node = motion_planner()
    rclpy.spin(motion_planner_node)

    motion_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()