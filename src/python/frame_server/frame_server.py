import socket
import struct
import cv2
from picamera2 import Picamera2

HOST = "0.0.0.0" # leave as is
PORT = 9000
JPEG_QUALITY = 90
RESOLUTION = (4608, 2592)

def capture_frame(picam2):
    frame = picam2.capture_array()  # RGB888
    h, w, _ = frame.shape

    ok, enc = cv2.imencode(".jpg", frame,
                           [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError("JPEG encode failed")

    return w, h, enc.tobytes()

def main():
    picam2 = Picamera2()

    picam2.configure(
        picam2.create_still_configuration(
            main={"size": RESOLUTION, "format": "RGB888"}
        )
    )

    picam2.start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"Created frame server with resolution ({RESOLUTION[0], RESOLUTION[1]})")
    print("Server listening...")

    while True:
        conn, addr = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        try:
            cmd = conn.recv(16)
            if cmd.strip() != b"SNAP":
                conn.close()
                continue

            print("Received frame request")

            w, h, payload = capture_frame(picam2)

            # Send header: width (I), height (I), payload size (Q)
            header = struct.pack("!IIQ", w, h, len(payload))

            conn.sendall(header)
            conn.sendall(payload)

        except Exception as e:
            print("Error:", e)

        finally:
            conn.close()

if __name__ == "__main__":
    main()