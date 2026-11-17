"""OpenCV access to an Arducam exposed as a Linux V4L2 video device.

FastAPI usage (create one shared camera at startup and close it at shutdown)::

    camera = Arducam()  # Find the Arducam regardless of its video number.

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
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np


def _device_identity(entry: Path) -> str:
    """Read V4L2 and USB product names without opening the video stream."""
    names = []
    for path in (entry / "name", *(parent / "product" for parent in
                                    (entry / "device").resolve().parents)):
        try:
            value = path.read_text().strip()
        except OSError:
            continue
        if value and value not in names:
            names.append(value)
    return " / ".join(names) or "unknown"


def _select_device(device: int | str | None) -> str:
    root = Path("/sys/class/video4linux")
    if device is None or device == "auto":
        entries = sorted(root.glob("video*"), key=lambda entry: int(entry.name[5:]))
    else:
        name = str(device)
        if name.isdecimal():
            name = f"video{name}"
        elif name.startswith("/dev/"):
            name = name[5:]
        if not name.startswith("video") or not name[5:].isdecimal():
            raise ValueError("device must be auto, a video index, videoN, or /dev/videoN")
        entries = [root / name]

    found = []
    for entry in entries:
        identity = _device_identity(entry)
        found.append(f"/dev/{entry.name}: {identity}")
        if "arducam" not in identity.casefold():
            continue
        # USB cameras can expose secondary metadata nodes; select the primary.
        try:
            if (entry / "index").read_text().strip() != "0":
                continue
        except OSError:
            continue
        return f"/dev/{entry.name}"
    raise RuntimeError(
        "No verified Arducam capture device found. "
        "The V4L2 or USB product name must contain 'Arducam'. Found: "
        + ("; ".join(found) or "no video devices")
    )


class Arducam:
    """Find and connect to an identified Arducam; call close() when finished.

    Width, height, and fps are requests; supported settings depend on the camera.
    This requires a camera/driver that exposes a standard V4L2 capture device.
    """

    MJPEG_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=frame"

    def __init__(
        self,
        device: int | str | None = None,
        *,
        width: int | None = None,
        height: int | None = None,
        fps: float = 30,
        jpeg_quality: int = 85,
    ):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be finite and positive")
        if not isinstance(jpeg_quality, int) or not 0 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be an integer between 0 and 100")
        for name, value in (("width", width), ("height", height)):
            if value is not None and (not isinstance(value, int) or value <= 0):
                raise ValueError(f"{name} must be a positive integer")

        self.device = _select_device(device)
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        try:
            if not self._capture.isOpened():
                raise RuntimeError(f"Could not open Arducam at {self.device}")
            # Request compressed capture before resolution/fps: YUYV can limit
            # this camera to 5 FPS at 1920x1080, while MJPEG supports 30 FPS.
            self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
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
            try:
                success, frame = self._capture.read()
            except cv2.error as exc:
                raise RuntimeError(
                    f"Could not decode a frame from {self.device}; check the USB connection"
                ) from exc
            if not success or frame is None or frame.size == 0:
                raise RuntimeError(f"Could not read a frame from {self.device}")
            return frame

    def detectChessboard(
        self,
        frame: np.ndarray | None = None,
        pattern_size: tuple[int, int] = (7, 7),
    ) -> np.ndarray | None:
        """Show detected inner corners and return an (N, 2) pixel array.

        pattern_size is (columns, rows) of inner corners, not squares.
        Coordinates follow OpenCV's row order; they are not chess square names
        or robot coordinates. Return None when the full pattern is not found.
        Call cv2.waitKey() in the preview loop to refresh the window.
        """
        if frame is None:
            frame = self.grabFrame()
        coordinates, preview = self._chessboard_result(frame, pattern_size)
        cv2.imshow("Chessboard detection", preview)
        return coordinates



    @staticmethod
    def _chessboard_result(
        frame: np.ndarray, pattern_size: tuple[int, int] = (7, 7),
    ) -> tuple[np.ndarray | None, np.ndarray]:
        """Detect and annotate without GUI calls, so workers can use it safely."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray, pattern_size,
            cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        preview = frame.copy()
        coordinates = None
        if found:
            coordinates = corners.reshape(-1, 2)
            cv2.drawChessboardCorners(preview, pattern_size, corners, found)
            for index, (x, y) in enumerate(coordinates):
                cv2.putText(
                    preview, f"{index}: ({x:.0f}, {y:.0f})",
                    (round(float(x)) + 6, round(float(y)) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA,
                )
        else:
            cv2.putText(
                preview, "Chessboard not detected", (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA,
            )
        return coordinates, preview

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
    parser.add_argument("--device", default="auto",
                        help="auto (default), video index, videoN, or /dev/videoN; must identify as Arducam")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=30)
    args = parser.parse_args()
    window_name = "Arducam"
    detector = ThreadPoolExecutor(max_workers=1, thread_name_prefix="chessboard")

    try:
        with Arducam(args.device, width=args.width, height=args.height, fps=args.fps) as camera:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, args.width, args.height)
            cv2.namedWindow("Chessboard detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Chessboard detection", args.width, args.height)
            fps_started = time.monotonic()
            frame_count = 0
            detection = None
            detection_preview = None
            while True:
                frame = camera.grabFrame()
                cv2.imshow(window_name, frame)
                if detection is not None and detection.done():
                    _, detection_preview = detection.result()
                    detection = None
                if detection is None:
                    # Only submit when idle: never accumulate stale frames.
                    detection = detector.submit(camera._chessboard_result, frame.copy())
                if detection_preview is None:
                    waiting_preview = frame.copy()
                    cv2.putText(
                        waiting_preview, "Detecting chessboard...", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA,
                    )
                    cv2.imshow("Chessboard detection", waiting_preview)
                else:
                    cv2.imshow("Chessboard detection", detection_preview)
                key = cv2.waitKey(1) & 0xFF
                frame_count += 1
                now = time.monotonic()
                elapsed = now - fps_started
                if elapsed >= 1:
                    print(f"Arducam preview: {frame_count / elapsed:.1f} FPS", flush=True)
                    fps_started = now
                    frame_count = 0
                if key in (ord("q"), ord("Q"), 27):
                    break
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if cv2.getWindowProperty("Chessboard detection", cv2.WND_PROP_VISIBLE) < 1:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        detector.shutdown(wait=True, cancel_futures=True)


if __name__ == "__main__":
    main()
