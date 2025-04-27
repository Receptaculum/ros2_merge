import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand
from std_msgs.msg import Int8MultiArray, UInt16
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import serial
import logging

import numpy as np

## <Parameter> #####################################################################################

# 통신 장치의 경로
PORT_NAME = "/dev/ttyACM0"

# Baud Rate
BAUD_RATE = 38400

# 노드 이름
NODE_NAME = "serial_communicator"

# 구독 토픽 이름
SUB_TOPIC_NAME = "command_data"

# 발행 토픽 이름
PUB_TOPIC_NAME = "arduino_data"

# Receiver 타이머 설정
TIMER = 0.1

# 로깅 여부
LOG = True

######################################################################################################


## <로그 출력> #########################################################################################
# DEBUG	self.get_logger().debug("msg")
# INFO	self.get_logger().info("msg")
# WARN	self.get_logger().warn("msg")
# ERROR	self.get_logger().error("msg")
# FATAL	self.get_logger().fatal("msg")
#######################################################################################################


## <QOS> ##############################################################################################
# Reliability : RELIABLE(신뢰성 보장), BEST_EFFORT(손실 감수, 최대한 빠른 전송)
# Durability : VOLATILE(전달한 메시지 제거), TRANSIENT_LOCAL(전달한 메시지 유지) / (Subscriber가 없을 때에 한함)
# History : KEEP_LAST(depth 만큼의 메시지 유지), KEEP_ALL(모두 유지)
# Liveliness : 활성 상태 감시
# Deadline : 최소 동작 보장 
#######################################################################################################


class serial_communicator(Node):
    def __init__(self, node_name, port_name, baud_rate, sub_topic_name, pub_topic_name, log):
        super().__init__(node_name)

        # Serial 통신을 위한 Class 선언
        self.serial = serial.Serial(port_name, baud_rate, rtscts=False)

        self.qos_sub = QoSProfile( # QOS 설정
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )
        
        self.qos_pub = QoSProfile( # QOS 설정
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )
        
        # 지령값 구독
        self.subscriber = self.create_subscription(MotionCommand, sub_topic_name, self.send_callback, self.qos_sub)

        # 아두이노 전송값 확인
        # self.timer = self.create_timer(TIMER, self.receive_callback)

        # 아두이노 전송값 배포
        self.publisher = self.create_publisher(UInt16, pub_topic_name, self.qos_pub)

        # 로깅 여부 설정
        if log == False: 
            self.get_logger().set_level(logging.FATAL)


    def send_callback(self, msg):
        # 값 크기 제한 및 Mapping
        steer_angle = int(np.clip(msg.steering, -128, 127)) 
        left_motor_speed = int(np.interp(msg.left_speed, [-255, 255], [-128, 127]))
        right_motor_speed = int(np.interp(msg.right_speed, [-255, 255], [-128, 127]))

        # 조향 s (0~255) 1 byte | 속도 v (0~255) 2 byte
        # bbbbbbbb(조향) bbbbbbbb(속도_좌) bbbbbbbb(속도_우)
        # 양수 -> 그대로 | 음수 -> 2의 보수 변환
        steer_angle_bin = hex(steer_angle if steer_angle > 0 else ((steer_angle+256) & 0xff))[2:].zfill(2)
        left_motor_speed_bin =  hex(left_motor_speed if left_motor_speed > 0 else ((left_motor_speed+256) & 0xff))[2:].zfill(2)
        right_motor_speed_bin =  hex(right_motor_speed if right_motor_speed > 0 else ((right_motor_speed+256) & 0xff))[2:].zfill(2)

        msg_hex = '0x' + steer_angle_bin + left_motor_speed_bin + right_motor_speed_bin
        msg = int(msg_hex, 16).to_bytes(3, byteorder='big', signed=False)  

        self.serial.write(msg)
        self.get_logger().info(f"TX: {msg}")


    def receive_callback(self):
        if self.serial.in_waiting >= 2:
            data = int.from_bytes(self.serial.read(2), byteorder='big', signed=False)
        
            msg = UInt16()
            msg.data = data
            self.publisher.publish(msg)
            self.get_logger().info(f"RX: {data}")


def main():
    rclpy.init()
    serial_node = serial_communicator(NODE_NAME, PORT_NAME, BAUD_RATE, SUB_TOPIC_NAME, PUB_TOPIC_NAME, LOG)
    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()