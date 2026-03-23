#!/usr/bin/env python3
#
# Debug node: publishes HSV mask, annotated image, and RViz 3D markers.
#
# Topics published:
#   /block_detector/debug/mask       — HSV binary mask (mono8)
#   /block_detector/debug/annotated  — color image with contour + centroid
#   /block_detector/debug/pose       — detected PoseStamped (camera frame)
#   /block_detector/debug/markers    — MarkerArray in link0 frame (RViz)

from __future__ import annotations

from collections import deque
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from block_grasp.block_detector import BlockDetector


BASE_FRAME = 'link0'


class BlockDetectorDebugNode(Node):

    def __init__(self) -> None:
        super().__init__('block_detector_debug_node')

        # HSV params
        self.declare_parameter('hue_min', 35)
        self.declare_parameter('hue_max', 85)
        self.declare_parameter('sat_min', 50)
        self.declare_parameter('sat_max', 255)
        self.declare_parameter('val_min', 50)
        self.declare_parameter('val_max', 255)
        self.declare_parameter('min_contour_area', 500)

        hsv_params = {k: self.get_parameter(k).value for k in [
            'hue_min', 'hue_max', 'sat_min', 'sat_max',
            'val_min', 'val_max', 'min_contour_area',
        ]}
        self._detector = BlockDetector(hsv_params)
        self._bridge = CvBridge()

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._color_img: np.ndarray | None = None
        self._depth_img: np.ndarray | None = None

        self.create_subscription(Image, '/camera/camera/color/image_rect_raw',
                                 self._color_cb, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw',
                                 self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self._info_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info',
                                 self._depth_info_cb, 10)

        self._pub_mask = self.create_publisher(Image, '/block_detector/debug/mask', 10)
        self._pub_anno = self.create_publisher(Image, '/block_detector/debug/annotated', 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/block_detector/debug/pose', 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/block_detector/debug/markers', 10
        )

        # Rolling average buffer (최근 10프레임)
        self._pose_buffer: deque = deque(maxlen=10)

        self.create_timer(0.1, self._timer_cb)  # 10 Hz

        self.get_logger().info(
            f'BlockDetectorDebugNode ready.\n'
            f'  Image  → /block_detector/debug/annotated\n'
            f'  Image  → /block_detector/debug/mask\n'
            f'  Marker → /block_detector/debug/markers  (Fixed Frame: {BASE_FRAME})'
        )

    def _color_cb(self, msg: Image) -> None:
        self._color_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_cb(self, msg: Image) -> None:
        self._depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._camera_info is None:
            self._camera_info = msg

    def _depth_info_cb(self, msg: CameraInfo) -> None:
        if self._depth_camera_info is None:
            self._depth_camera_info = msg

    def _timer_cb(self) -> None:
        if self._color_img is None or self._depth_img is None:
            return

        color = self._color_img.copy()
        depth = self._depth_img.copy()

        # ── HSV mask ──────────────────────────────────────────────────────
        h_min = self.get_parameter('hue_min').value
        h_max = self.get_parameter('hue_max').value
        s_min = self.get_parameter('sat_min').value
        s_max = self.get_parameter('sat_max').value
        v_min = self.get_parameter('val_min').value
        v_max = self.get_parameter('val_max').value
        min_area = self.get_parameter('min_contour_area').value

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([h_min, s_min, v_min], dtype=np.uint8),
            np.array([h_max, s_max, v_max], dtype=np.uint8),
        )

        # ── Annotated image ───────────────────────────────────────────────
        annotated = color.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        cx_px = cy_px = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) >= min_area:
                detected = True
                cv2.drawContours(annotated, [largest], -1, (0, 255, 0), 2)
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx_px = int(M['m10'] / M['m00'])
                    cy_px = int(M['m01'] / M['m00'])
                    cv2.circle(annotated, (cx_px, cy_px), 6, (0, 0, 255), -1)
                    cv2.putText(
                        annotated,
                        f'({cx_px}, {cy_px})  area={int(cv2.contourArea(largest))}',
                        (cx_px + 10, cy_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
                    )

        label = 'DETECTED' if detected else 'NOT DETECTED'
        cv2.putText(annotated, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0) if detected else (0, 0, 255), 2)

        # ── Publish images ────────────────────────────────────────────────
        now = self.get_clock().now().to_msg()

        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header.stamp = now
        self._pub_mask.publish(mask_msg)

        # ── 3D detection + TF + markers ───────────────────────────────────
        if not detected or self._camera_info is None or self._depth_camera_info is None:
            anno_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            anno_msg.header.stamp = now
            self._pub_anno.publish(anno_msg)
            self._publish_no_detection_marker()
            return

        # BlockDetector로 3D 역투영 (depth intrinsics 사용)
        depth_cam_info = self._depth_camera_info
        depth_cam_info.header.stamp = now
        pose_cam = self._detector.detect(color, depth, depth_cam_info)
        if pose_cam is None:
            self._publish_no_detection_marker()
            return

        # PoseStamped (camera frame) 퍼블리시
        pose_cam.header.stamp = now
        self._pub_pose.publish(pose_cam)

        # TF → BASE_FRAME (stamp=0 → 가장 최근 transform 사용)
        pose_cam.header.stamp.sec = 0
        pose_cam.header.stamp.nanosec = 0
        try:
            pose_base: PoseStamped = self._tf_buffer.transform(
                pose_cam,
                BASE_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=2.0)
            self._publish_no_detection_marker()
            return

        raw_x = pose_base.pose.position.x
        raw_y = pose_base.pose.position.y
        raw_z = pose_base.pose.position.z

        # ── Rolling average ───────────────────────────────────────────────
        self._pose_buffer.append((raw_x, raw_y, raw_z))
        arr = np.array(self._pose_buffer)
        bx, by, bz = arr.mean(axis=0)

        # 이미지에 smoothed 좌표 표시
        cv2.putText(
            annotated,
            f'smooth: ({bx:.3f}, {by:.3f}, {bz:.3f})',
            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1,
        )
        anno_msg2 = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        anno_msg2.header.stamp = now
        self._pub_anno.publish(anno_msg2)

        # ── Publish RViz markers ──────────────────────────────────────────
        markers = MarkerArray()

        # 블록 위치 (raw): 반투명 빨간 구
        sphere_raw = Marker()
        sphere_raw.header.frame_id = BASE_FRAME
        sphere_raw.header.stamp = now
        sphere_raw.ns = 'block_raw'
        sphere_raw.id = 0
        sphere_raw.type = Marker.SPHERE
        sphere_raw.action = Marker.ADD
        sphere_raw.pose = pose_base.pose
        sphere_raw.scale.x = sphere_raw.scale.y = sphere_raw.scale.z = 0.03
        sphere_raw.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.4)
        sphere_raw.lifetime.sec = 1
        markers.markers.append(sphere_raw)

        # 블록 위치 (smoothed): 진한 빨간 구
        sphere = Marker()
        sphere.header.frame_id = BASE_FRAME
        sphere.header.stamp = now
        sphere.ns = 'block'
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = bx
        sphere.pose.position.y = by
        sphere.pose.position.z = bz
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
        sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        sphere.lifetime.sec = 1
        markers.markers.append(sphere)

        # 텍스트 레이블
        txt = Marker()
        txt.header.frame_id = BASE_FRAME
        txt.header.stamp = now
        txt.ns = 'block_label'
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = bx
        txt.pose.position.y = by
        txt.pose.position.z = bz + 0.07
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.03
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        txt.text = f'x={bx:.3f}\ny={by:.3f}\nz={bz:.3f}'
        txt.lifetime.sec = 1
        markers.markers.append(txt)

        self._pub_markers.publish(markers)

    def _publish_no_detection_marker(self) -> None:
        """감지 실패 시 마커 삭제."""
        markers = MarkerArray()
        for mid, ns in [(0, 'block'), (1, 'block_label')]:
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            m.action = Marker.DELETE
            markers.markers.append(m)
        self._pub_markers.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockDetectorDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
