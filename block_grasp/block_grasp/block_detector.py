#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo


class BlockDetector:
    """
    Detects a colored block using HSV filtering and returns its 3-D pose
    in camera_color_optical_frame via depth back-projection.
    """

    def __init__(self, hsv_params: dict) -> None:
        self._h_min: int = int(hsv_params['hue_min'])
        self._h_max: int = int(hsv_params['hue_max'])
        self._s_min: int = int(hsv_params['sat_min'])
        self._s_max: int = int(hsv_params['sat_max'])
        self._v_min: int = int(hsv_params['val_min'])
        self._v_max: int = int(hsv_params['val_max'])
        self._min_area: int = int(hsv_params['min_contour_area'])

    def detect(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: CameraInfo,
    ) -> PoseStamped | None:
        """
        Detect the largest color-matching block and return its 3-D pose.

        Args:
            color_image: BGR uint8 H×W×3 image.
            depth_image: Depth image. uint16 → millimetres, float32 → metres.
                         (RealSense D405 default: uint16 mm)
            camera_info: Intrinsics for the color stream.

        Returns:
            PoseStamped in camera_color_optical_frame, or None if not found.
        """
        # ── HSV mask ──────────────────────────────────────────────────────
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower = np.array([self._h_min, self._s_min, self._v_min], dtype=np.uint8)
        upper = np.array([self._h_max, self._s_max, self._v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # ── Contour detection ─────────────────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self._min_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0.0:
            return None

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        # ── 5×5 median depth (noise reduction) ───────────────────────────
        h, w = depth_image.shape[:2]
        r0, r1 = max(0, v - 2), min(h, v + 3)
        c0, c1 = max(0, u - 2), min(w, u + 3)
        patch = depth_image[r0:r1, c0:c1].astype(np.float32)
        valid = patch[patch > 0]
        if valid.size == 0:
            return None

        z_raw = float(np.median(valid))

        # D405 default: uint16 in millimetres → convert to metres.
        # If the stream is already float32 metres, use as-is.
        if depth_image.dtype == np.uint16:
            z = z_raw / 1000.0
        else:
            z = z_raw

        if z <= 0.0 or z > 2.0:
            return None

        # ── Intrinsic back-projection ─────────────────────────────────────
        fx: float = camera_info.k[0]
        fy: float = camera_info.k[4]
        cx: float = camera_info.k[2]
        cy: float = camera_info.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # ── Build PoseStamped ─────────────────────────────────────────────
        pose = PoseStamped()
        pose.header.frame_id = 'camera_depth_optical_frame'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # identity — orientation resolved after TF

        return pose
