"""
Hand Detector Base Class

손 검출 및 관절 각도 추정을 위한 추상 인터페이스를 정의합니다.
새로운 검출기 구현체는 이 클래스를 상속받아야 합니다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np


@dataclass
class HandLandmark:
    """단일 랜드마크 좌표."""
    x: float
    y: float
    z: float
    visibility: float = 1.0


@dataclass
class JointAngles:
    """손가락 관절 각도 (도 단위, 0-180)."""
    thumb_cmc: float = 0.0
    thumb_mcp: float = 0.0
    thumb_ip: float = 0.0

    index_mcp: float = 0.0
    index_pip: float = 0.0
    index_dip: float = 0.0

    middle_mcp: float = 0.0
    middle_pip: float = 0.0
    middle_dip: float = 0.0

    ring_mcp: float = 0.0
    ring_pip: float = 0.0
    ring_dip: float = 0.0

    pinky_mcp: float = 0.0
    pinky_pip: float = 0.0
    pinky_dip: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        """딕셔너리로 변환."""
        return {
            'thumb_cmc': self.thumb_cmc,
            'thumb_mcp': self.thumb_mcp,
            'thumb_ip': self.thumb_ip,
            'index_mcp': self.index_mcp,
            'index_pip': self.index_pip,
            'index_dip': self.index_dip,
            'middle_mcp': self.middle_mcp,
            'middle_pip': self.middle_pip,
            'middle_dip': self.middle_dip,
            'ring_mcp': self.ring_mcp,
            'ring_pip': self.ring_pip,
            'ring_dip': self.ring_dip,
            'pinky_mcp': self.pinky_mcp,
            'pinky_pip': self.pinky_pip,
            'pinky_dip': self.pinky_dip,
        }

    def to_list(self) -> List[float]:
        """순서대로 리스트로 변환 (모델 입력용)."""
        return list(self.to_dict().values())


@dataclass
class DetectionResult:
    """손 검출 결과."""
    detected: bool = False
    handedness: Optional[str] = None  # 'Left' or 'Right'
    confidence: float = 0.0
    landmarks: List[HandLandmark] = field(default_factory=list)
    joint_angles: Optional[JointAngles] = None

    def to_dict(self) -> Dict:
        """직렬화 가능한 딕셔너리로 변환."""
        return {
            'detected': self.detected,
            'handedness': self.handedness,
            'confidence': self.confidence,
            'landmarks': [
                {'x': lm.x, 'y': lm.y, 'z': lm.z, 'visibility': lm.visibility}
                for lm in self.landmarks
            ],
            'joint_angles': self.joint_angles.to_dict() if self.joint_angles else {}
        }


class BaseHandDetector(ABC):
    """
    손 검출 및 관절 각도 추정을 위한 추상 베이스 클래스.

    새로운 검출기를 구현하려면 이 클래스를 상속받고
    detect()와 release() 메서드를 구현해야 합니다.

    Example:
        class MyDetector(BaseHandDetector):
            def detect(self, image):
                # 구현
                return DetectionResult(...)

            def release(self):
                # 리소스 해제
                pass
    """

    @abstractmethod
    def detect(self, image: np.ndarray) -> DetectionResult:
        """
        입력 이미지에서 손을 검출하고 관절 각도를 추정합니다.

        Args:
            image: OpenCV BGR 포맷의 이미지 (H, W, 3) np.uint8

        Returns:
            DetectionResult: 검출 결과
                - detected: 손 검출 여부
                - handedness: 왼손/오른손
                - confidence: 검출 신뢰도
                - landmarks: 21개 랜드마크 좌표 (정규화됨 0-1)
                - joint_angles: 15개 관절 각도 (도 단위)

        Raises:
            ValueError: 유효하지 않은 이미지 입력
        """
        pass

    @abstractmethod
    def release(self) -> None:
        """
        검출기가 사용하는 리소스를 해제합니다.

        GPU 메모리, 파일 핸들 등을 정리할 때 사용합니다.
        노드 종료 시 반드시 호출해야 합니다.
        """
        pass

    def __enter__(self):
        """Context manager 진입."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager 종료 시 리소스 해제."""
        self.release()
        return False
