import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import cv2
import numpy as np

import math


## <Parameter> #######################################################################################

# 구독 토픽 이름
SUB_TOPIC_NAME_CART = 'lidar_cartesian'
SUB_TOPIC_NAME_TF = 'lidar_data'

# 출력 화면 크기 (가로, 세로)
SIZE = [900, 900]

# 최소 거리 [m]
MIN = 0.1 

# 최대 거리 [m]
MAX = 1.0

# 최소 각도
MIN_ANGLE = 0

# 최대 각도
MAX_ANGLE = 30

# 확장 계수
K = 400

######################################################################################################


## <LIDAR> ###########################################################################################
#        270
#       #######  (Motor)
#     0 # 본체 ######### 180     (Counter-Clockwise)
#       #######
#         90
#
#
#       <--x    y      - 부호      y  x-->
#               |      ---->>     |
#               v                 v
######################################################################################################


class lidar_debugger(Node):
    def __init__(self):
        super().__init__("lidar_debugger")

        # QOS 선언
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publisher / Subscriber 선언
        self.subscriber_cart = self.create_subscription(Float32MultiArray, SUB_TOPIC_NAME_CART, self.disp_callback, self.qos_profile)
        self.subscriber_tf = self.create_subscription(Bool, SUB_TOPIC_NAME_TF, self.tf_callback, self.qos_profile)

        # Bool 저장을 위한 레지스터 선언
        self.bool = False


    def tf_callback(self, msg):
        self.bool = msg.data


    def disp_callback(self, msg):
        msg = np.array(msg.data).reshape(-1, 2).tolist()
        background = np.zeros([SIZE[1], SIZE[0]])

        center_x = int(SIZE[0]/2)
        center_y = int(SIZE[1]/2)

        for x, y in msg:
            try:
                background[int(y*K + center_y)][int(x*K + center_x)] = 1
            except:
                pass
        
        cv2.circle(background, [center_x, center_y], int(MIN*K), 255, thickness=1)
        cv2.circle(background, [center_x, center_y], int(MAX*K), 255, thickness=1)

        cv2.line(img = background, 
                 pt1 = [center_x, center_y],
                 pt2 = [int((-math.cos(MIN_ANGLE*math.pi/180)*SIZE[0] + center_x)),
                        int((math.sin(MIN_ANGLE*math.pi/180)*SIZE[1] + center_y))], 
                 color = 255, 
                 thickness=1) 

        cv2.line(img = background, 
                 pt1 = [center_x, center_y],
                 pt2 = [int((-math.cos(MAX_ANGLE*math.pi/180)*SIZE[0] + center_x)),
                        int((math.sin(MAX_ANGLE*math.pi/180)*SIZE[1] + center_y))], 
                 color = 255, 
                 thickness=1) 

        if self.bool == True:
            (_, h), _ = cv2.getTextSize(text = "Detected",
                                        fontFace = cv2.FONT_HERSHEY_COMPLEX, 
                                        fontScale=1,
                                        thickness=1)

            cv2.putText(img = background,
                        text = "Detected",
                        org=[5, 5+h],
                        fontFace=cv2.FONT_HERSHEY_COMPLEX,
                        fontScale=1,
                        color=255,
                        thickness=1)

        # background = cv2.resize(background, (800, 800))

        cv2.imshow("LIDAR", background)
        cv2.waitKey(5)


def main():
    rclpy.init()
    lidar_debugger_node = lidar_debugger()

    rclpy.spin(lidar_debugger_node)

    lidar_debugger_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()