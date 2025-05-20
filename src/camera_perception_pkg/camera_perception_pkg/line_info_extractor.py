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
from interfaces_pkg.msg import SegmentGroup, LineData

import logging
import numpy as np


## <Parameter> #######################################################################################

# 구독 토픽 이름
SUB_TOPIC_NAME = "segmented_data_rear"

# 배포 토픽 이름
PUB_TOPIC_NAME = "line_data_rear"

# 로깅 여부
LOG = True

# CV 처리 영상 출력 여부
DEBUG = True

# 영상 크기
FRAME_SIZE = [640, 480]

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
        self.publisher = self.create_publisher(LineData, self.pub_topic, self.qos_profile)
    
        # 로깅 여부 설정
        if LOG == False: 
            self.get_logger().set_level(logging.FATAL)


    def yolov8_detections_callback(self, msg):
        # 메시지 선언
        result = LineData()

        # 빈 이미지 생성
        img_base = np.zeros([FRAME_SIZE[1], FRAME_SIZE[0]]).astype(np.uint8)
        img_hough = np.zeros([FRAME_SIZE[1], FRAME_SIZE[0]]).astype(np.uint8)
        img_hough_post = np.zeros([FRAME_SIZE[1], FRAME_SIZE[0]]).astype(np.uint8)

        # 점 데이터
        point = np.array(msg.line).reshape(-1, 2).astype(np.int32)

        # 점 데이터 연결 및 이미지에 투사
        cv2.polylines(img_base, [point], isClosed=False, color=255, thickness=2)

        # Hough 변환
        lines = cv2.HoughLinesP(
            img_base, rho=1, theta=np.pi/180, threshold=50, 
            minLineLength=75, maxLineGap=100
        )

        # 분류 결과 저장 레지스터
        line_l = []
        line_r = []
        line_c = []

        # 거리 정보 및 각도 정보 저장 레지스터
        d1 = []
        d2 = []
        grad = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                val = np.array([[x1, y1], [x2, y2]])

                # y축 기준 내림차순 정렬
                val = val[val[:, 1].argsort()[::-1]]            
                
                # 거리 및 각도 계산
                d1.append(abs(val[0][0] - 0) + abs(val[0][1] - 480))
                d2.append(abs(val[0][0] - 640) + abs(val[0][1] - 480))
                grad.append(np.arctan(abs(y1 - y2)/abs(x1 - x2 + 1e-6)) * 180 / np.pi)

                # 선 삽입
                cv2.line(img_hough, (x1, y1), (x2, y2), 255, 1)

            # 인덱스 추출
            d1_idx = d1.index(min(d1))
            d2_idx = d2.index(min(d2))
            grad_idx = grad.index(min(grad))

            if grad[grad_idx] < 10:
                line_c.extend(lines[grad_idx][0])

            if d1_idx == d2_idx:
                pass

            else:
                if grad[d1_idx] > 20 and d1_idx != grad_idx:
                    line_l.extend(lines[d1_idx][0])
    
                if grad[d2_idx] > 20 and d2_idx != grad_idx:
                    line_r.extend(lines[d2_idx][0])
        
        # 감지 결과 출력을 위한 String
        detection = ""

        # 좌우 차선의 길이를 평준화하기 위한 조건
        if line_l != [] and line_r != []:
            x1_l, y1_l, x2_l, y2_l = line_l
            x1_r, y1_r, x2_r, y2_r = line_r

            y_max = max(y1_l, y2_l, y1_r, y2_r)
            y_min = min(y1_l, y2_l, y1_r, y2_r)

            grad_l = (x2_l - x1_l)/(y2_l - y1_l + 1e-6)
            grad_r = (x2_r - x1_r)/(y2_r - y1_r + 1e-6)

            new_x1_l = grad_l*(y_max - y1_l) + x1_l
            new_x2_l = grad_l*(y_min - y1_l) + x1_l

            new_x1_r = grad_r*(y_max - y1_r) + x1_r
            new_x2_r = grad_r*(y_min - y1_r) + x1_r

            line_l = [int(new_x1_l), y_max, int(new_x2_l), y_min]
            line_r = [int(new_x1_r), y_max, int(new_x2_r), y_min]


        if line_l != []:
            x1, y1, x2, y2 = line_l
            cv2.line(img_hough_post, (x1, y1), (x2, y2), 255, 1)
            detection += "L"

        if line_r != []:
            x1, y1, x2, y2 = line_r
            cv2.line(img_hough_post, (x1, y1), (x2, y2), 255, 1) 
            detection += "R"

        if line_c != []:
            x1, y1, x2, y2 = line_c
            cv2.line(img_hough_post, (x1, y1), (x2, y2), 255, 1) 
            detection += "C"


        # 글자 길이 확인
        (_, h), _ = cv2.getTextSize(text = detection,
                                    fontFace = cv2.FONT_HERSHEY_COMPLEX, 
                                    fontScale=1,
                                    thickness=1)

        # 글자 삽입
        cv2.putText(img = img_hough_post,
                    text = detection,
                    org=[5, 5+h],
                    fontFace=cv2.FONT_HERSHEY_COMPLEX,
                    fontScale=1,
                    color=255,
                    thickness=1)     

        # 이미지 출력
        if DEBUG ==  True:
            img_concat = np.concatenate((img_base, img_hough, img_hough_post), axis=1)
            img_concat = cv2.resize(img_concat, (1280, 320))
            cv2.imshow("LINE", img_concat)
            cv2.waitKey(1)

        # 결과값 할당
        result.left = list(map(int, line_l))
        result.right = list(map(int, line_r))
        result.center = list(map(int, line_c))

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