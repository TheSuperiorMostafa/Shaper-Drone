"""
UDP stream viewer for ShaperDrone camera feed.
Receives JPEG frames over UDP from shapeVideo.py and displays them.

Usage:
  python udp_viewer.py [port]

Example:
  python udp_viewer.py 8081

Make sure shapeVideo.py is running with ENABLE_UDP_STREAM=True.
"""

import sys
import socket
import struct
import cv2
import numpy as np

DEFAULT_PORT = 8081


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(1.0)

    print(f"[UDP Viewer] Listening on port {port}")
    print("[UDP Viewer] Press 'q' to quit")
    print("[UDP Viewer] Ensure shapeVideo.py is running with ENABLE_UDP_STREAM=True")

    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break

        if len(data) < 4:
            continue

        # Parse: [4-byte size][JPEG data]
        try:
            frame_size = struct.unpack('>I', data[:4])[0]
            jpeg_data = np.frombuffer(data[4:4 + frame_size], dtype=np.uint8)

            frame = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("ShaperDrone UDP Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            pass  # Skip malformed packets

    sock.close()
    cv2.destroyAllWindows()
    print("[UDP Viewer] Closed")


if __name__ == "__main__":
    main()
