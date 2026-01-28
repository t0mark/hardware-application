"""
MediaPipe Hand Detector Implementation

MediaPipe Tasks API를 사용한 손 검출 및 관절 각도 추정 구현체.
(MediaPipe 0.10.x+ 호환)
"""

import cv2
import numpy as np
from typing import List, Optional
import os

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from .base_detector import (
    BaseHandDetector,
    DetectionResult,
    HandLandmark,
    JointAngles
)


class MediaPipeHandDetector(BaseHandDetector):
    """MediaPipe Tasks API를 사용한 손 검출 구현체."""

    # 모델 다운로드 URL
    MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
    MODEL_PATH = "/tmp/hand_landmarker.task"

    def __init__(
        self,
        static_image_mode: bool = False,
        max_num_hands: int = 1,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5
    ):
        """
        MediaPipe Hand Detector를 초기화합니다.

        Args:
            static_image_mode: True면 IMAGE 모드, False면 VIDEO 모드
            max_num_hands: 검출할 최대 손 개수
            min_detection_confidence: 검출 신뢰도 임계값 (0.0 - 1.0)
            min_tracking_confidence: 트래킹 신뢰도 임계값 (0.0 - 1.0)
        """
        # 모델 파일 다운로드
        self._download_model_if_needed()

        # 실행 모드 설정
        running_mode = vision.RunningMode.IMAGE if static_image_mode else vision.RunningMode.VIDEO

        # HandLandmarker 옵션 설정
        base_options = python.BaseOptions(model_asset_path=self.MODEL_PATH)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=running_mode,
            num_hands=max_num_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._running_mode = running_mode
        self._frame_timestamp = 0
        self._initialized = True

    def _download_model_if_needed(self):
        """모델 파일이 없으면 다운로드합니다."""
        if os.path.exists(self.MODEL_PATH):
            return

        import urllib.request
        print(f"Downloading hand landmarker model to {self.MODEL_PATH}...")
        urllib.request.urlretrieve(self.MODEL_URL, self.MODEL_PATH)
        print("Download complete.")

    def detect(self, image: np.ndarray) -> DetectionResult:
        """
        입력 이미지에서 손을 검출하고 관절 각도를 추정합니다.

        Args:
            image: OpenCV BGR 포맷 이미지 (H, W, 3)

        Returns:
            DetectionResult: 검출 결과
        """
        if not self._initialized or image is None:
            return DetectionResult(detected=False)

        # BGR -> RGB 변환
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # MediaPipe Image로 변환
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)

        # 검출 수행
        if self._running_mode == vision.RunningMode.IMAGE:
            results = self._landmarker.detect(mp_image)
        else:
            # VIDEO 모드는 timestamp 필요
            self._frame_timestamp += 33  # ~30fps
            results = self._landmarker.detect_for_video(mp_image, self._frame_timestamp)

        if not results.hand_landmarks:
            return DetectionResult(detected=False)

        # 첫 번째 손 처리
        hand_landmarks = results.hand_landmarks[0]

        # 랜드마크 추출 (21개)
        landmarks = self._extract_landmarks(hand_landmarks)

        # 손잡이 정보
        handedness = None
        confidence = 0.0
        if results.handedness:
            hand_info = results.handedness[0][0]
            handedness = hand_info.category_name  # 'Left' or 'Right'
            confidence = hand_info.score

        # 관절 각도 계산
        joint_angles = self._compute_finger_angles(landmarks)

        return DetectionResult(
            detected=True,
            handedness=handedness,
            confidence=confidence,
            landmarks=landmarks,
            joint_angles=joint_angles
        )

    def _extract_landmarks(self, hand_landmarks) -> List[HandLandmark]:
        """MediaPipe 랜드마크를 HandLandmark 리스트로 변환."""
        landmarks = []
        for lm in hand_landmarks:
            landmarks.append(HandLandmark(
                x=lm.x,
                y=lm.y,
                z=lm.z,
                visibility=getattr(lm, 'visibility', 1.0)
            ))
        return landmarks

    def _compute_finger_angles(self, landmarks: List[HandLandmark]) -> JointAngles:
        """
        손가락 굴곡 각도를 계산합니다.

        MediaPipe Hand Landmarks:
        0: WRIST
        1-4: THUMB (CMC, MCP, IP, TIP)
        5-8: INDEX (MCP, PIP, DIP, TIP)
        9-12: MIDDLE (MCP, PIP, DIP, TIP)
        13-16: RING (MCP, PIP, DIP, TIP)
        17-20: PINKY (MCP, PIP, DIP, TIP)
        """
        pts = np.array([[lm.x, lm.y, lm.z] for lm in landmarks])

        return JointAngles(
            # Thumb
            thumb_cmc=self._calc_angle(pts[0], pts[1], pts[2]),
            thumb_mcp=self._calc_angle(pts[1], pts[2], pts[3]),
            thumb_ip=self._calc_angle(pts[2], pts[3], pts[4]),
            # Index
            index_mcp=self._calc_angle(pts[0], pts[5], pts[6]),
            index_pip=self._calc_angle(pts[5], pts[6], pts[7]),
            index_dip=self._calc_angle(pts[6], pts[7], pts[8]),
            # Middle
            middle_mcp=self._calc_angle(pts[0], pts[9], pts[10]),
            middle_pip=self._calc_angle(pts[9], pts[10], pts[11]),
            middle_dip=self._calc_angle(pts[10], pts[11], pts[12]),
            # Ring
            ring_mcp=self._calc_angle(pts[0], pts[13], pts[14]),
            ring_pip=self._calc_angle(pts[13], pts[14], pts[15]),
            ring_dip=self._calc_angle(pts[14], pts[15], pts[16]),
            # Pinky
            pinky_mcp=self._calc_angle(pts[0], pts[17], pts[18]),
            pinky_pip=self._calc_angle(pts[17], pts[18], pts[19]),
            pinky_dip=self._calc_angle(pts[18], pts[19], pts[20])
        )

    def _calc_angle(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
        """
        세 점으로 이루어진 각도 계산 (p2가 꼭짓점).
        180도 = 펴진 상태, 0도 = 구부러진 상태
        """
        v1 = p1 - p2
        v2 = p3 - p2

        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)

        if norm1 < 1e-6 or norm2 < 1e-6:
            return 180.0

        v1 = v1 / norm1
        v2 = v2 / norm2

        dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
        return float(np.degrees(np.arccos(dot)))

    def get_finger_curl(self, landmarks: List[HandLandmark]) -> dict:
        """
        손가락 굽힘 정도를 0-1 사이 값으로 정규화하여 반환합니다.
        0: 완전히 펴짐, 1: 완전히 구부러짐
        """
        angles = self._compute_finger_angles(landmarks)

        def normalize(angle: float, min_angle: float = 60.0, max_angle: float = 180.0) -> float:
            curl = (max_angle - angle) / (max_angle - min_angle)
            return float(np.clip(curl, 0.0, 1.0))

        return {
            'thumb': normalize((angles.thumb_mcp + angles.thumb_ip) / 2),
            'index': normalize((angles.index_pip + angles.index_dip) / 2),
            'middle': normalize((angles.middle_pip + angles.middle_dip) / 2),
            'ring': normalize((angles.ring_pip + angles.ring_dip) / 2),
            'pinky': normalize((angles.pinky_pip + angles.pinky_dip) / 2)
        }

    def draw_hand(self, image: np.ndarray, landmarks: List[HandLandmark]) -> np.ndarray:
        """이미지에 손 랜드마크와 연결선을 그립니다."""
        output = image.copy()
        h, w = image.shape[:2]

        # 랜드마크 좌표를 픽셀로 변환
        points = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]

        # 연결선 정의
        connections = [
            # Thumb
            (0, 1), (1, 2), (2, 3), (3, 4),
            # Index
            (0, 5), (5, 6), (6, 7), (7, 8),
            # Middle
            (0, 9), (9, 10), (10, 11), (11, 12),
            # Ring
            (0, 13), (13, 14), (14, 15), (15, 16),
            # Pinky
            (0, 17), (17, 18), (18, 19), (19, 20),
            # Palm
            (5, 9), (9, 13), (13, 17)
        ]

        # 연결선 그리기
        for start, end in connections:
            cv2.line(output, points[start], points[end], (0, 255, 0), 2)

        # 랜드마크 점 그리기
        for i, pt in enumerate(points):
            color = (255, 0, 0) if i == 0 else (0, 0, 255)  # 손목은 파란색
            cv2.circle(output, pt, 5, color, -1)

        return output

    def release(self) -> None:
        """MediaPipe 리소스를 해제합니다."""
        if self._initialized and hasattr(self, '_landmarker'):
            self._landmarker.close()
            self._initialized = False
