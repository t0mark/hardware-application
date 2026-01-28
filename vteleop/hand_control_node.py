#!/usr/bin/env python3
"""
Control Node for Inspire Hand

Perception 노드의 출력을 Inspire Hand 하드웨어 제어 토픽으로 리매핑합니다.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from service_interfaces.msg import SetAngle1


class InspireHandControlNode(Node):
    """Perception 결과를 Inspire Hand 제어 명령으로 변환하는 노드."""

    # Inspire Hand 손가락 ID 매핑
    # 1: Pinky, 2: Ring, 3: Middle, 4: Index, 5: Thumb_Flex, 6: Thumb_Abd
    FINGER_IDS = {
        'pinky': 1,
        'ring': 2,
        'middle': 3,
        'index': 4,
        'thumb_flex': 5,
        'thumb_abd': 6,
    }

    def __init__(self):
        super().__init__('inspire_hand_control_node')

        # 파라미터 선언
        self.declare_parameter('input_topic', '/hand/joint_angles')
        self.declare_parameter('output_topic', 'set_angle_data')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('angle_min', 0)
        self.declare_parameter('angle_max', 1000)
        self.declare_parameter('smoothing_factor', 0.3)
        self.declare_parameter('enable_thumb_abd', True)
        self.declare_parameter('target_hand', 'right')  # 'left', 'right', 'any'

        # 파라미터 가져오기
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.enable_thumb_abd = self.get_parameter('enable_thumb_abd').value
        self.target_hand = self.get_parameter('target_hand').value.lower()

        # 상태 변수
        self.last_angles = {}  # 스무딩을 위한 이전 각도 저장
        self.latest_perception = None  # 최신 perception 데이터

        # Subscriber
        self.perception_sub = self.create_subscription(
            String,
            input_topic,
            self.perception_callback,
            10
        )

        # Publisher
        self.angle_pub = self.create_publisher(SetAngle1, output_topic, 10)

        # Timer for rate-limited publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_control)

        self.get_logger().info(f'Inspire Hand Control Node started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Target hand: {self.target_hand}')

    def perception_callback(self, msg: String):
        """Perception 데이터 콜백."""
        try:
            self.latest_perception = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f'Failed to parse perception data: {e}')

    def publish_control(self):
        """제어 명령 발행."""
        if self.latest_perception is None:
            return

        data = self.latest_perception

        if not data.get('detected', False):
            return

        # 타겟 손 필터링
        handedness = data.get('handedness', '').lower()
        if self.target_hand != 'any':
            if handedness != self.target_hand:
                return

        # Joint angles에서 손가락별 굴곡 정도 계산
        joint_angles = data.get('joint_angles', {})
        if not joint_angles:
            return

        # 손가락별 제어 각도 계산
        finger_angles = self._compute_finger_angles(joint_angles)

        # 스무딩 적용
        smoothed_angles = self._apply_smoothing(finger_angles)

        # SetAngle1 메시지 생성 및 발행
        msg = SetAngle1()
        for finger_name, angle in smoothed_angles.items():
            finger_id = self.FINGER_IDS.get(finger_name)
            if finger_id is not None:
                # Thumb_abd는 옵션
                if finger_name == 'thumb_abd' and not self.enable_thumb_abd:
                    continue
                msg.finger_ids.append(finger_id)
                msg.angles.append(int(angle))

        if msg.finger_ids:
            self.angle_pub.publish(msg)
            self.get_logger().debug(
                f'Published angles: {list(zip(msg.finger_ids, msg.angles))}'
            )

    def _compute_finger_angles(self, joint_angles: dict) -> dict:
        """
        Joint angles를 Inspire Hand 제어 각도로 변환.

        MediaPipe angles: 180 = extended, ~60 = flexed
        Inspire Hand: 0 = open/extended, 1000 = closed/flexed
        """
        result = {}

        # Pinky: pip + dip 평균
        pinky_angle = self._avg_angle(
            joint_angles.get('pinky_pip', 180),
            joint_angles.get('pinky_dip', 180)
        )
        result['pinky'] = self._map_angle_to_inspire(pinky_angle)

        # Ring: pip + dip 평균
        ring_angle = self._avg_angle(
            joint_angles.get('ring_pip', 180),
            joint_angles.get('ring_dip', 180)
        )
        result['ring'] = self._map_angle_to_inspire(ring_angle)

        # Middle: pip + dip 평균
        middle_angle = self._avg_angle(
            joint_angles.get('middle_pip', 180),
            joint_angles.get('middle_dip', 180)
        )
        result['middle'] = self._map_angle_to_inspire(middle_angle)

        # Index: pip + dip 평균
        index_angle = self._avg_angle(
            joint_angles.get('index_pip', 180),
            joint_angles.get('index_dip', 180)
        )
        result['index'] = self._map_angle_to_inspire(index_angle)

        # Thumb Flex: mcp + ip 평균
        thumb_flex_angle = self._avg_angle(
            joint_angles.get('thumb_mcp', 180),
            joint_angles.get('thumb_ip', 180)
        )
        result['thumb_flex'] = self._map_angle_to_inspire(thumb_flex_angle)

        # Thumb Abd: cmc (opposition)
        thumb_abd_angle = joint_angles.get('thumb_cmc', 180)
        result['thumb_abd'] = self._map_angle_to_inspire(thumb_abd_angle)

        return result

    def _avg_angle(self, angle1: float, angle2: float) -> float:
        """두 각도의 평균."""
        return (angle1 + angle2) / 2.0

    def _map_angle_to_inspire(
        self,
        angle: float,
        min_angle: float = 60.0,
        max_angle: float = 180.0
    ) -> int:
        """
        MediaPipe 각도를 Inspire Hand 각도로 매핑.

        MediaPipe: 180 (extended) -> 60 (flexed)
        Inspire: 0 (open) -> 1000 (closed)
        """
        # 각도를 0-1 범위로 정규화 (180=0, 60=1)
        normalized = (max_angle - angle) / (max_angle - min_angle)
        normalized = max(0.0, min(1.0, normalized))

        # Inspire Hand 범위로 스케일
        inspire_angle = int(normalized * (self.angle_max - self.angle_min) + self.angle_min)
        return max(self.angle_min, min(self.angle_max, inspire_angle))

    def _apply_smoothing(self, angles: dict) -> dict:
        """지수 이동 평균을 사용한 스무딩."""
        smoothed = {}
        alpha = self.smoothing_factor

        for finger, angle in angles.items():
            if finger in self.last_angles:
                # EMA: new = alpha * current + (1 - alpha) * previous
                smoothed[finger] = alpha * angle + (1 - alpha) * self.last_angles[finger]
            else:
                smoothed[finger] = angle
            self.last_angles[finger] = smoothed[finger]

        return smoothed


def main(args=None):
    """메인 함수."""
    rclpy.init(args=args)
    node = InspireHandControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
