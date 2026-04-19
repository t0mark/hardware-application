"""
ZED2i 카메라 MJPEG 스트리밍 서버 (OpenCV / UVC 모드)

ZED2i는 UVC 규격으로 side-by-side 스테레오 프레임을 출력합니다.
좌측 절반(LEFT eye)만 잘라서 JPEG로 인코딩해 스트리밍합니다.

Usage:
    python camera.py --port 8080 --fps 30 --quality 80 --resolution HD720
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2

# ZED2i UVC 해상도: (full_width, height) — side-by-side이므로 left = full_width // 2
RESOLUTION_MAP = {
    "HD2K":   (4416, 1242),
    "HD1080": (3840, 1080),
    "HD720":  (2560,  720),
    "VGA":    (1344,  376),
}

# ---------------------------------------------------------------------------
# 전역 프레임 버퍼
# ---------------------------------------------------------------------------
_frame_lock = threading.Lock()
_latest_jpeg: bytes = b""


def camera_worker(device_index: int, resolution: str, fps: int, quality: int):
    """OpenCV로 ZED2i UVC 스트림을 읽어 JPEG로 인코딩하는 백그라운드 스레드."""
    global _latest_jpeg

    full_w, h = RESOLUTION_MAP.get(resolution, RESOLUTION_MAP["HD720"])
    left_w = full_w // 2

    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  full_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS,          fps)

    if not cap.isOpened():
        print(f"[Camera] 카메라 열기 실패 (device {device_index})")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[Camera] 시작: {actual_w}x{actual_h} @ {actual_fps}fps (device {device_index})")

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[Camera] 프레임 읽기 실패, 재시도...")
            time.sleep(0.1)
            continue

        # side-by-side에서 좌측 eye만 잘라냄
        left = frame[:, : actual_w // 2]

        _, jpeg = cv2.imencode(".jpg", left, encode_params)
        with _frame_lock:
            _latest_jpeg = jpeg.tobytes()

    cap.release()


# ---------------------------------------------------------------------------
# MJPEG HTTP 핸들러
# ---------------------------------------------------------------------------
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
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
    parser = argparse.ArgumentParser(description="ZED2i Camera MJPEG Streaming Server (OpenCV/UVC)")
    parser.add_argument("--port",       type=int,   default=8080,  help="HTTP 포트 (기본값: 8080)")
    parser.add_argument("--device",     type=int,   default=0,     help="카메라 장치 번호 (기본값: 0)")
    parser.add_argument("--fps",        type=int,   default=30,    help="카메라 FPS (기본값: 30)")
    parser.add_argument("--quality",    type=int,   default=80,    help="JPEG 품질 0-100 (기본값: 80)")
    parser.add_argument("--resolution", type=str,   default="HD720",
                        choices=list(RESOLUTION_MAP.keys()),
                        help="카메라 해상도 (기본값: HD720)")
    args = parser.parse_args()

    cam_thread = threading.Thread(
        target=camera_worker,
        args=(args.device, args.resolution, args.fps, args.quality),
        daemon=True,
    )
    cam_thread.start()

    print("[Server] 첫 프레임 대기 중...")
    start = time.time()
    while not _latest_jpeg:
        if time.time() - start > 10:
            print("[Server] 카메라 초기화 타임아웃")
            return
        time.sleep(0.1)

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
