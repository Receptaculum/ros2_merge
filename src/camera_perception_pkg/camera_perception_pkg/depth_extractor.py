import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image

import cv2
import cv_bridge

import logging

from .lib.depth_estimator.depth_estimator import depth_estimator


## <Parameter> #######################################################################################

# 구독 토픽 이름
SUB_TOPIC_NAME = "image_publisher"

# 배포 토픽 이름
PUB_TOPIC_NAME = "none"

# 로깅 여부
LOG = True

######################################################################################################


class DepthExtractor(Node):
    def __init__(self):
        super().__init__('car_info_extractor')

        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        #self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.subscriber = self.create_subscription(Image, self.sub_topic, self.depth_estimation_callback, self.qos_profile)
        #self.publisher = self.create_publisher(CarData, self.pub_topic, self.qos_profile)
        
        # CV Bridge Object 선언
        self.bridge = cv_bridge.CvBridge() 

        # 로깅 여부 설정
        if LOG == False: 
            self.get_logger().set_level(logging.FATAL)


    def depth_estimation_callback(self, msg: Image):
        # 깊이 추정 결과 출력
        frame = depth_estimator(self.bridge.imgmsg_to_cv2(msg))
        frame = cv2.normalize(frame, None, 0, 1, cv2.NORM_MINMAX)
 
        print(frame)
        cv2.imshow("DEPTH", frame)
        cv2.waitKey(5)


def main(args=None):
    rclpy.init(args=args)
    node = DepthExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
  
  
if __name__ == '__main__':
    main()
