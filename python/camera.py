"""
ZED 카메라 MJPEG 스트리밍 서버

Usage:
    python camera_server.py --port 8080 --fps 30 --quality 80 --resolution HD720
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import pyzed.sl as sl


# ---------------------------------------------------------------------------
# 전역 프레임 버퍼
# ---------------------------------------------------------------------------
_frame_lock = threading.Lock()
_latest_jpeg: bytes = b""


def camera_worker(resolution: str, fps: int, quality: int):
    """ZED 카메라에서 프레임을 읽어 JPEG로 인코딩하는 백그라운드 스레드."""
    global _latest_jpeg

    resolution_map = {
        "HD2K":   sl.RESOLUTION.HD2K,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD720":  sl.RESOLUTION.HD720,
        "VGA":    sl.RESOLUTION.VGA,
    }

    camera = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = resolution_map.get(resolution, sl.RESOLUTION.HD720)
    init_params.camera_fps = fps
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # 깊이 불필요 → 성능 향상

    status = camera.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[Camera] ZED 카메라 열기 실패: {status}")
        return

    print(f"[Camera] ZED 카메라 시작 ({resolution}, {fps}fps)")

    runtime_params = sl.RuntimeParameters()
    mat = sl.Mat()
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

    while True:
        if camera.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            camera.retrieve_image(mat, sl.VIEW.LEFT)
            frame = mat.get_data()  # BGRA numpy array

            # BGRA → BGR 변환 후 JPEG 인코딩
            bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            _, jpeg = cv2.imencode(".jpg", bgr, encode_params)

            with _frame_lock:
                _latest_jpeg = jpeg.tobytes()

    camera.close()


# ---------------------------------------------------------------------------
# MJPEG HTTP 핸들러
# ---------------------------------------------------------------------------
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # 요청 로그 억제 (너무 많이 찍힘)
        pass

    def do_GET(self):
        if self.path == "/stream":
            self._stream()
        elif self.path == "/snapshot":
            self._snapshot()
        else:
            self.send_error(404)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        print(f"[Server] 클라이언트 연결: {self.client_address}")
        try:
            while True:
                with _frame_lock:
                    jpeg = _latest_jpeg

                if jpeg:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                else:
                    time.sleep(0.01)
        except (BrokenPipeError, ConnectionResetError):
            print(f"[Server] 클라이언트 연결 종료: {self.client_address}")

    def _snapshot(self):
        """단일 JPEG 스냅샷 반환 (/snapshot)"""
        with _frame_lock:
            jpeg = _latest_jpeg

        if not jpeg:
            self.send_error(503, "No frame available")
            return

        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)


# ---------------------------------------------------------------------------
# 진입점
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="ZED Camera MJPEG Streaming Server")
    parser.add_argument("--port",       type=int,   default=8080,   help="HTTP 포트 (기본값: 8080)")
    parser.add_argument("--fps",        type=int,   default=30,     help="카메라 FPS (기본값: 30)")
    parser.add_argument("--quality",    type=int,   default=80,     help="JPEG 품질 0-100 (기본값: 80)")
    parser.add_argument("--resolution", type=str,   default="HD720",
                        choices=["HD2K", "HD1080", "HD720", "VGA"],
                        help="카메라 해상도 (기본값: HD720)")
    args = parser.parse_args()

    # 카메라 스레드 시작
    cam_thread = threading.Thread(
        target=camera_worker,
        args=(args.resolution, args.fps, args.quality),
        daemon=True,
    )
    cam_thread.start()

    # 첫 프레임 대기
    print("[Server] 첫 프레임 대기 중...")
    timeout = 10
    start = time.time()
    while not _latest_jpeg:
        if time.time() - start > timeout:
            print("[Server] 카메라 초기화 타임아웃")
            return
        time.sleep(0.1)

    # HTTP 서버 시작
    server = HTTPServer(("0.0.0.0", args.port), MJPEGHandler)
    print(f"[Server] 스트리밍 시작: http://0.0.0.0:{args.port}/stream")
    print(f"[Server] 스냅샷:        http://0.0.0.0:{args.port}/snapshot")
    print("[Server] 종료: Ctrl+C")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Server] 종료")
        server.shutdown()


if __name__ == "__main__":
    main()
