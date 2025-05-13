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

from std_msgs.msg import String
from interfaces_pkg.msg import SegmentGroup

import logging
import numpy as np


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
        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)
    
        # 로깅 여부 설정
        if LOG == False: 
            self.get_logger().set_level(logging.FATAL)


    def yolov8_detections_callback(self, msg):
        result = String()
        processed_data = dict()
        line_data = np.array(msg.line).reshape(-1, 2)

        for k in range(0, 480, 30):
            try:
                processed_data[k] = len(line_data[(k < line_data[:, 1]) & (line_data[:, 1] <= k + 30)])
            except:
                processed_data[k] = 0

        self.get_logger().info(f"ss: {max(processed_data, key=processed_data.get)}")
        


        ###### 코드 작성 요구 #####

        # 결과 Publish
        self.publisher.publish(result)


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