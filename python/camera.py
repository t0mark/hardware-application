"""
ZED2i 카메라 ZMQ 스트리밍 서버 (OpenCV / UVC 모드)

Usage:
    python camera.py --port 5556 --fps 30 --quality 80 --resolution HD720
"""

import argparse
import threading
import time

import cv2
import zmq

RESOLUTION_MAP = {
    "HD2K":   (4416, 1242),
    "HD1080": (3840, 1080),
    "HD720":  (2560,  720),
    "VGA":    (1344,  376),
}

_frame_lock = threading.Lock()
_latest_jpeg: bytes = b""


def camera_worker(device_index: int, resolution: str, fps: int, quality: int):
    global _latest_jpeg

    full_w, h = RESOLUTION_MAP.get(resolution, RESOLUTION_MAP["HD720"])

    device_path = f"/dev/video{device_index}" if isinstance(device_index, int) else device_index
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  full_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS,          fps)

    if not cap.isOpened():
        print(f"[Camera] 카메라 열기 실패 (device {device_index})")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] 시작: {actual_w}x{actual_h} @ {fps}fps")

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        left = frame[:, : actual_w // 2]
        _, jpeg = cv2.imencode(".jpg", left, encode_params)
        with _frame_lock:
            _latest_jpeg = jpeg.tobytes()


def main():
    parser = argparse.ArgumentParser(description="ZED2i Camera ZMQ Streaming Server")
    parser.add_argument("--port",       type=int,   default=5556)
    parser.add_argument("--device",     type=int,   default=0)
    parser.add_argument("--fps",        type=int,   default=30)
    parser.add_argument("--quality",    type=int,   default=80)
    parser.add_argument("--resolution", type=str,   default="HD720",
                        choices=list(RESOLUTION_MAP.keys()))
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

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)  # 최신 프레임만 유지
    socket.bind(f"tcp://0.0.0.0:{args.port}")
    print(f"[Server] ZMQ 스트리밍 시작: tcp://0.0.0.0:{args.port}")
    print("[Server] 종료: Ctrl+C")

    try:
        while True:
            with _frame_lock:
                jpeg = _latest_jpeg
            if jpeg:
                socket.send(jpeg, zmq.NOBLOCK)
            time.sleep(1 / args.fps)
    except KeyboardInterrupt:
        print("\n[Server] 종료")
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
