###########
# 후방 전용 #
###########

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Float32MultiArray
from interfaces_pkg.msg import SegmentGroup

import logging


## <Parameter> #######################################################################################

# 구독 토픽 이름
SUB_TOPIC_NAME = "segmented_data_rear"

# 배포 토픽 이름
PUB_TOPIC_NAME = "line_data_rear"

# 로깅 여부
LOG = True

######################################################################################################


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_info_extractor_rear')

        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.subscriber = self.create_subscription(SegmentGroup, self.sub_topic, self.yolov8_detections_callback, self.qos_profile)
        self.publisher = self.create_publisher(Float32MultiArray, self.pub_topic, self.qos_profile)
    
        # 로깅 여부 설정
        if LOG == False: 
            self.get_logger().set_level(logging.FATAL)


    def yolov8_detections_callback(self, msg):
        line = Float32MultiArray()

        print(msg)
        ###### 코드 작성 요구 #####

        # 결과 Publish
        self.publisher.publish(line)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
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