import time
import socket
import struct
from datetime import datetime

PI_IP = "10.0.0.20"  
PORT = 9000

def recvn(sock, n):
    data = b""
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            raise ConnectionError("Connection closed")
        data += packet
    return data

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    sock.connect((PI_IP, PORT))
    sock.sendall(b"SNAP")

    startTime = time.time()

    # Read header
    header = recvn(sock, 4 + 4 + 8)
    w, h, size = struct.unpack("!IIQ", header)

    # Read image
    payload = recvn(sock, size)
    sock.close()

    stopTime = time.time()
    timeDelta = stopTime - startTime
    print(f"Received frame in {timeDelta} seconds")

    filename = f"frame_{w}x{h}_{datetime.now().strftime('%H%M%S')}.jpg"

    with open(filename, "wb") as f:
        f.write(payload)

    print("Saved:", filename)

if __name__ == "__main__":
    main()