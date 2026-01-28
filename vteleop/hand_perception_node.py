#!/usr/bin/env python3
"""
Hand Perception Node for ROS2

손 검출 및 관절 각도 추정을 위한 ROS2 노드.
카메라 이미지를 구독하고 손 관절 정보를 퍼블리시합니다.
"""

import cv2
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from vteleop.detectors import MediaPipeHandDetector


class HandPerceptionNode(Node):
    """손 인식 ROS2 노드."""

    def __init__(self):
        super().__init__('hand_perception_node')

        # 파라미터 선언
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/hand/joint_angles')
        self.declare_parameter('viz_topic', '/hand/viz_image')
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('max_hands', 1)
        self.declare_parameter('enable_viz', True)

        # 파라미터 가져오기
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        viz_topic = self.get_parameter('viz_topic').value
        detection_conf = self.get_parameter('detection_confidence').value
        tracking_conf = self.get_parameter('tracking_confidence').value
        max_hands = self.get_parameter('max_hands').value
        self.enable_viz = self.get_parameter('enable_viz').value

        # CV Bridge 초기화
        self.bridge = CvBridge()

        # 손 검출기 초기화
        try:
            self.detector = MediaPipeHandDetector(
                static_image_mode=False,
                max_num_hands=max_hands,
                min_detection_confidence=detection_conf,
                min_tracking_confidence=tracking_conf
            )
            self.get_logger().info('MediaPipe Hand Detector initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize detector: {e}')
            self.detector = None

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.joint_pub = self.create_publisher(String, output_topic, 10)
        self.viz_pub = self.create_publisher(Image, viz_topic, 10)

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(f'Publishing joints to: {output_topic}')
        self.get_logger().info(f'Publishing viz to: {viz_topic}')

    def image_callback(self, msg: Image):
        """이미지 콜백 함수."""
        if self.detector is None:
            return

        try:
            # ROS Image -> OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 손 검출
            result = self.detector.detect(cv_image)

            # 관절 각도 퍼블리시
            output_msg = String()
            output_msg.data = json.dumps(result.to_dict())
            self.joint_pub.publish(output_msg)

            # 시각화 이미지 퍼블리시
            if self.enable_viz:
                viz_image = self._create_viz_image(cv_image, result)
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)

            if result.detected:
                self.get_logger().debug(
                    f"Hand detected - Handedness: {result.handedness}"
                )

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _create_viz_image(self, image, result):
        """
        검출 결과를 이미지에 오버레이합니다.

        Args:
            image: 원본 OpenCV 이미지
            result: DetectionResult

        Returns:
            시각화된 이미지
        """
        viz = image.copy()

        if not result.detected:
            # 손 미검출 시 상태 표시
            cv2.putText(
                viz, "No hand detected", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )
            return viz

        # 랜드마크 그리기
        viz = self.detector.draw_hand(viz, result.landmarks)

        # 손잡이 정보 표시
        hand_text = f"{result.handedness} ({result.confidence:.2f})"
        cv2.putText(
            viz, hand_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        # Finger curl 값 표시 (로봇 제어용)
        curl = self.detector.get_finger_curl(result.landmarks)
        y_offset = 60
        for finger, value in curl.items():
            text = f"{finger}: {value:.2f}"
            cv2.putText(
                viz, text, (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
            )
            y_offset += 20

        return viz

    def destroy_node(self):
        """노드 종료 시 리소스 정리."""
        if self.detector is not None:
            self.detector.release()
        super().destroy_node()


def main(args=None):
    """메인 함수."""
    rclpy.init(args=args)
    node = HandPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
