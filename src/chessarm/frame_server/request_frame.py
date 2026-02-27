import time
import socket
import struct
import time
import numpy as np
import cv2
from datetime import datetime

PI_IP = "10.0.0.20"  
PORT = 9000

class FrameRequester:

    def __init__(self):

        self.lastresponse = None



    def recvn(self, sock, n):
        data = b""
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                raise ConnectionError("Connection closed")
            data += packet
        return data



    def requestFrameToJPG(self):

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        sock.connect((PI_IP, PORT))
        sock.sendall(b"SNAP")

        startTime = time.time()

        # Read header
        header = self.recvn(sock, 4 + 4 + 8)
        w, h, size = struct.unpack("!IIQ", header)

        # Read image
        payload = self.recvn(sock, size)
        sock.close()

        stopTime = time.time()
        print(f"Received frame in {stopTime - startTime:.4f} seconds")

        filename = f"frame_{w}x{h}_{datetime.now().strftime('%H%M%S')}.jpg"

        with open(filename, "wb") as f:
            f.write(payload)

        print("Saved:", filename)



    def requestFrameToMat(self, timeout=2.0):

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(timeout)

        try:
            sock.connect((PI_IP, PORT))
            sock.sendall(b"SNAP")

            startTime = time.time()

            header = self.recvn(sock, 4 + 4 + 8)
            w, h, size = struct.unpack("!IIQ", header)

            payload = self.recvn(sock, size)

            stopTime = time.time()
            print(f"Received frame in {stopTime - startTime:.4f} seconds")

            np_arr = np.frombuffer(payload, dtype=np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is None:
                self.lastResponse = "bad image"
                raise RuntimeError("Failed to decode JPEG payload")
            
            self.lastResponse = "good"

            return image

        except socket.timeout:
            print("Frame request timed out")
            self.lastResponse = "timeout"
            return None

        except (ConnectionRefusedError, OSError) as e:
            print(f"Connection error: {e}")
            self.lastResponse = "connection refused"
            return None

        finally:
            sock.close()




def main():

    frameRequester = FrameRequester()

    frameRequester.requestFrameToJPG()


    

if __name__ == "__main__":
    main()