"""OpenCV access to an Arducam exposed as a Linux V4L2 video device.

FastAPI usage (create one shared camera at startup and close it at shutdown)::

    camera = Arducam(device=0)

    @app.get("/camera/stream")
    def camera_stream():
        return StreamingResponse(camera.mjpeg_frames(),
                                 media_type=Arducam.MJPEG_CONTENT_TYPE)

The frontend can display that endpoint with ``<img src="/camera/stream" />``.
Import StreamingResponse from fastapi.responses. Use a synchronous route so
camera reads run off the async event loop. Multiple callers share serialized
camera reads; each call captures a new frame rather than a cached image.
"""

from __future__ import annotations

import math
import argparse
import threading
import time
from collections.abc import Iterator

import cv2
import numpy as np


class Arducam:
    """Connect immediately to video0 or video1; call close() when finished.

    Width, height, and fps are requests; supported settings depend on the camera.
    This requires a camera/driver that exposes a standard V4L2 capture device.
    """

    MJPEG_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=frame"

    def __init__(
        self,
        device: int | str = 0,
        *,
        width: int | None = None,
        height: int | None = None,
        fps: float = 30,
        jpeg_quality: int = 85,
    ):
        devices = {0: "/dev/video0", 1: "/dev/video1",
                   "video0": "/dev/video0", "video1": "/dev/video1",
                   "/dev/video0": "/dev/video0", "/dev/video1": "/dev/video1"}
        if device not in devices:
            raise ValueError("device must be 0, 1, video0, video1, /dev/video0, or /dev/video1")
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be finite and positive")
        if not isinstance(jpeg_quality, int) or not 0 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be an integer between 0 and 100")
        for name, value in (("width", width), ("height", height)):
            if value is not None and (not isinstance(value, int) or value <= 0):
                raise ValueError(f"{name} must be a positive integer")

        self.device = devices[device]
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        try:
            if not self._capture.isOpened():
                raise RuntimeError(f"Could not open Arducam at {self.device}")
            if width is not None:
                self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height is not None:
                self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._capture.set(cv2.CAP_PROP_FPS, fps)
        except Exception:
            self._capture.release()
            raise

    def grabFrame(self) -> np.ndarray:
        """Capture a BGR image; raise RuntimeError on closure or capture failure."""
        with self._lock:
            if self._closed.is_set():
                raise RuntimeError("Arducam is closed")
            success, frame = self._capture.read()
            if not success or frame is None or frame.size == 0:
                raise RuntimeError(f"Could not read a frame from {self.device}")
            return frame

    def jpeg_frame(self) -> bytes:
        """Capture one frame encoded as JPEG for an image/jpeg response."""
        success, encoded = cv2.imencode(
            ".jpg", self.grabFrame(),
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not success:
            raise RuntimeError("Could not encode Arducam frame as JPEG")
        return encoded.tobytes()

    def mjpeg_frames(self) -> Iterator[bytes]:
        """Yield multipart JPEG chunks until closed or the consumer disconnects.

        Closing one iterator leaves the shared camera open for other consumers.
        Capture/encoding failures propagate to the server instead of retrying
        forever. The configured fps limits each stream's output rate.
        """
        while not self._closed.is_set():
            started = time.monotonic()
            try:
                jpeg = self.jpeg_frame()
            except RuntimeError:
                if self._closed.is_set():
                    return
                raise
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n")
            self._closed.wait(max(0, 1 / self.fps - (time.monotonic() - started)))

    def close(self) -> None:
        """Release the device. Safe to call multiple times."""
        with self._lock:
            if not self._closed.is_set():
                self._closed.set()
                self._capture.release()

    def __enter__(self) -> Arducam:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def main() -> None:
    """Preview the camera; exit with Q, Escape, Ctrl+C, or the window close button."""
    parser = argparse.ArgumentParser(description="Display live Arducam frames")
    parser.add_argument("--device", type=int, choices=(0, 1), default=0)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=30)
    args = parser.parse_args()
    window_name = "Arducam"

    try:
        with Arducam(args.device, width=args.width, height=args.height, fps=args.fps) as camera:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, args.width, args.height)
            while True:
                cv2.imshow(window_name, camera.grabFrame())
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
