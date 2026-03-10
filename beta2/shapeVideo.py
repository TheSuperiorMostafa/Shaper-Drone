# Transitioning from static images to video shape detection

import os
import sys

# Set OpenCV to use a backend that doesn't require X11 (for headless operation)
if 'DISPLAY' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    # Try to set OpenCV backend to avoid Qt issues
    try:
        os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'
    except:
        pass

import cv2
import time
import numpy as np
import threading
import socket
import struct
from queue import Queue, Empty
from http.server import BaseHTTPRequestHandler, HTTPServer
from navigation import FlightController
from shape_navigator import ShapeNavigator
from tof_sensor import TOFSensor
from remote_control import RemoteControl

# Vision-only mode: skip flight controller connection (for standalone Pi vision testing)
# Set to True when testing camera/shape detection without FC connected; False when flying.
VISION_ONLY = True

# Check if running headless (no display)
HEADLESS_MODE = 'DISPLAY' not in os.environ
if HEADLESS_MODE:
    print("[INFO] Running in headless mode (no display available)")

# Debug flag for verbose shape-order debounce logging
DEBUG_DEBOUNCE = False

# Stream / processing latency
# - LOW_RES: 320x240 (faster but text/order gets cut off; set False for readable overlays)
# - PROCESS_LATEST_ONLY: skip backlog, always process/display newest frame (reduces delay)
# - PROCESS_EVERY_N_FRAMES: only process every Nth frame (2 = process every 2nd frame, reduces CPU load)
# - ENABLE_HTTP_STREAM: enable HTTP streaming server (disable to reduce CPU/network overhead)
# - ENABLE_UDP_STREAM: enable UDP streaming (lower latency than HTTP)
# - UDP_TARGET_IP: laptop/viewer IP for unicast (e.g. '10.47.232.91'). Set None for broadcast (may not work on eduroam)
LOW_RES_STREAM = False  # 640x480 so full order and labels fit on screen
PROCESS_LATEST_ONLY = True
PROCESS_EVERY_N_FRAMES = 2  # Process every 2nd frame to reduce CPU load
ENABLE_HTTP_STREAM = False  # Set False to disable HTTP streaming and reduce lag
ENABLE_UDP_STREAM = True  # Set True to enable UDP streaming
UDP_TARGET_IP = '10.47.14.1'  # Set to your laptop IP (e.g. '10.47.232.91') for unicast; None = broadcast

# Try to import picamera2 for Raspberry Pi camera
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False
    print("[INFO] picamera2 not available, will use OpenCV VideoCapture (USB camera)")


shape_order = []      # will hold up to 3 unique shapes, e.g. ["triangle", "square", "circle"]
order_complete = False

# Debounce state
current_candidate = None
candidate_start_time = None
DEBOUNCE_SECONDS = 1.5  # require ~3s of stable viewing to add a shape to the array

# Global variable to hold the latest frame for streaming
latest_frame = None
frame_lock = threading.Lock()

# Thread-safe queues for frame processing pipeline
# Smaller queues (1-2) reduce latency by preventing frame accumulation
# Frames are dropped if processing can't keep up, ensuring stream shows recent frames
raw_frame_queue = Queue(maxsize=1)  # Raw frames from camera - keep small for low latency
processed_frame_queue = Queue(maxsize=1)  # Processed frames - keep small for low latency
shutdown_event = threading.Event()  # Signal to stop all threads

# Set by stdin listener thread when user types in terminal (headless)
reset_from_stdin_event = threading.Event()
stop_from_stdin_event = threading.Event()

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with frame_lock:
                    if latest_frame is not None:
                        # Encode frame as JPEG - lower quality (55) for faster encoding and smaller files
                        # This improves stream responsiveness on slower hardware
                        ret, buffer = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
                        if ret:
                            frame_bytes = buffer.tobytes()
                            self.wfile.write(b'--frame\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(frame_bytes)))
                            self.end_headers()
                            self.wfile.write(frame_bytes)
                            self.wfile.write(b'\r\n')
                # Reduced sleep for higher frame rate - actual rate limited by camera + processing
                # If processing is slow, we'll show frames as fast as they're available
                time.sleep(0.01)
        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'''
<!DOCTYPE html>
<html>
<head>
    <title>ShaperDrone Camera Stream</title>
    <style>
        body { margin: 0; background: #000; text-align: center; }
        img { max-width: 100%; height: auto; }
    </style>
</head>
<body>
    <img src="/stream" alt="Camera Stream">
</body>
</html>
            ''')
        else:
            self.send_error(404)

def start_streaming_server(port=8080):
    """Start HTTP server for MJPEG streaming"""
    server = HTTPServer(('0.0.0.0', port), StreamingHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"[STREAM] Camera stream available at http://<pi_ip>:{port}/stream")
    print(f"[STREAM] Web viewer at http://<pi_ip>:{port}/")
    return server

def udp_stream_thread(port=8081, shutdown_event=None, target_ip=None):
    """
    UDP streaming thread - sends JPEG frames over UDP for lower latency.
    target_ip: send to this IP (unicast). If None, use broadcast (may not work on eduroam).
    View with: python udp_viewer.py
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if target_ip:
        dest = (target_ip, port)
        print(f"[UDP] UDP stream started, sending to {target_ip}:{port} (unicast)")
    else:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        dest = ('255.255.255.255', port)
        print(f"[UDP] UDP stream started on port {port} (broadcast)")
    print(f"[UDP] Run: python udp_viewer.py (on the viewer machine)")
    
    while not shutdown_event.is_set():
        try:
            with frame_lock:
                if latest_frame is not None:
                    # Encode frame as JPEG (same quality as HTTP stream)
                    ret, buffer = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        frame_size = len(frame_bytes)
                        
                        # Send frame size (4 bytes) + frame data
                        # Format: [4-byte size][JPEG data]
                        packet = struct.pack('>I', frame_size) + frame_bytes
                        
                        try:
                            sock.sendto(packet, dest)
                        except OSError:
                            if not target_ip:
                                try:
                                    sock.sendto(packet, ('127.0.0.1', port))
                                except OSError:
                                    pass
            
            # Small sleep to avoid hammering the network
            time.sleep(0.01)
        except Exception as e:
            print(f"[UDP] Stream error: {e}")
            time.sleep(0.1)
    
    sock.close()
    print("[UDP] UDP stream stopped")

def frame_capture_thread(camera, use_picamera, raw_frame_queue, shutdown_event):
    """
    Thread function to continuously capture frames from camera.
    Runs independently to prevent blocking the main loop.
    """
    print("[THREAD] Frame capture thread started")
    consecutive_errors = 0
    max_errors = 5
    
    while not shutdown_event.is_set():
        try:
            # Read frame from camera
            if use_picamera:
                frame = camera.capture_array()
                # picamera2 returns RGB, convert to BGR for OpenCV
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = camera.read()
                if not ret:
                    consecutive_errors += 1
                    if consecutive_errors >= max_errors:
                        print("[THREAD] Frame capture thread: too many read errors, stopping")
                        break
                    time.sleep(0.1)
                    continue
            
            consecutive_errors = 0
            
            # Put frame in queue for processing (non-blocking, drop old frames if queue is full)
            try:
                raw_frame_queue.put_nowait(frame)
            except:
                # Queue is full, drop oldest frame and add new one
                try:
                    raw_frame_queue.get_nowait()
                    raw_frame_queue.put_nowait(frame)
                except:
                    pass
            
        except Exception as e:
            print(f"[THREAD] Frame capture error: {e}")
            time.sleep(0.1)
    
    print("[THREAD] Frame capture thread stopped")

def frame_processing_thread(raw_frame_queue, processed_frame_queue, tof_sensor, shutdown_event, min_contour_area=800):
    """
    Thread function to process frames for shape detection.
    Runs independently to prevent blocking frame capture or navigation.
    min_contour_area: minimum contour area (e.g. 200 for 320x240, 800 for 640x480).
    """
    print("[THREAD] Frame processing thread started")
    
    # Frame skipping counter for PROCESS_EVERY_N_FRAMES
    frame_counter = 0
    last_result = None  # Store last processed result to reuse when skipping frames
    
    while not shutdown_event.is_set():
        try:
            # Get frame(s) from queue
            frame = None
            if PROCESS_LATEST_ONLY:
                # Drain queue and keep only the latest frame (reduces delay from backlog)
                while True:
                    try:
                        frame = raw_frame_queue.get_nowait()
                    except Empty:
                        break
            if frame is None:
                try:
                    frame = raw_frame_queue.get(timeout=0.05)
                except Empty:
                    continue
            
            # Frame skipping: only process every Nth frame to reduce CPU load
            frame_counter += 1
            if frame_counter % PROCESS_EVERY_N_FRAMES != 0:
                # Skip processing - reuse last result if available, or pass raw frame
                if last_result is not None:
                    # Reuse last processed result (labels/positions stay same, but update frame for visual continuity)
                    result = last_result.copy()
                    result['frame'] = frame  # Use current raw frame so stream stays responsive
                    try:
                        processed_frame_queue.put_nowait(result)
                    except:
                        try:
                            processed_frame_queue.get_nowait()
                            processed_frame_queue.put_nowait(result)
                        except:
                            pass
                continue
            
            # Read ToF sensor distance
            distance = tof_sensor.read_distance()
            
            # Process frame for shape detection
            annotated, frame_labels, shape_positions = process_frame(frame, min_contour_area)
            
            # Store result for reuse when skipping frames
            last_result = {
                'frame': annotated,
                'labels': frame_labels,
                'positions': shape_positions,
                'distance': distance
            }
            
            # Put processed result in queue
            try:
                processed_frame_queue.put_nowait(last_result)
            except:
                # Queue is full, drop oldest result and add new one
                try:
                    processed_frame_queue.get_nowait()
                    processed_frame_queue.put_nowait(last_result)
                except:
                    pass
                    
        except Exception as e:
            print(f"[THREAD] Frame processing error: {e}")
            time.sleep(0.01)
    
    print("[THREAD] Frame processing thread stopped")

def stdin_listener_thread(shutdown_evt, reset_evt, stop_evt):
    """
    Read lines from terminal stdin; set reset_evt for 'r'/'reset', stop_evt for 's'/'stop'.
    Allows reset and emergency stop from the SSH terminal when running headless.
    """
    while not shutdown_evt.is_set():
        try:
            line = sys.stdin.readline()
            if not line:
                break
            s = line.strip().lower()
            if s in ('r', 'reset'):
                reset_evt.set()
            elif s in ('s', 'stop'):
                stop_evt.set()
        except Exception:
            break
    print("[THREAD] Stdin listener stopped")

def classify(contour, min_area=800):
    area = cv2.contourArea(contour)

    if area < min_area:
        return None

    perim = cv2.arcLength(contour, True)
    if perim == 0:
        return None

    circularity = 4.0 * np.pi * area / (perim * perim)

    # For checking how well the circle fills its minimum enclosing circle
    (_, _),radius = cv2.minEnclosingCircle(contour) # minEnclosingCircle returns ((cx,cy), radius)
    circle_area = np.pi * radius * radius
    fill_ratio = area / circle_area if circle_area > 0 else 0

    eps = 0.01 * perim
    approx = cv2.approxPolyDP(contour, eps, True)
    n = len(approx)

    # --------------- Circle test first -------------------------------
    # For a real disk, both circularity and fill_ratio should be high.
    # (These thresholds are intentionally not super strict to handle low-contrast edges.)
    if n>= 7 and circularity > 0.75 and fill_ratio > 0.7:
        return "Circle"
    
    # -------------- Polygon approximation for non-circles --------------
    if n == 3:
        return "Triangle"
    elif n == 4:
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        return "Square" if 0.9 < ar < 1.1 else "Rectangle"
    elif n == 5:
        # For a "real" pentagon we expect something not crazy skinny (closer to a square)
        # Aspect ratio (ar) and circularity checks added so long skinny shapes aren't detected
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        if 0.7 < ar < 1.3 and circularity > 0.3:
            return "Pentagon"
        else:
            return None
    elif n == 6:
        # Same goes for a hexagon
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        if 0.7 < ar < 1.3 and circularity > 0.3:
            return "Hexagon"
        else:
            return None
    else: 
        return None


def process_frame(image, min_contour_area=800):
    """
    Takes a BGR image, returns:
      - annotated_image
      - list of labels detected in this frame (e.g. ["Triangle", "Circle"])
      - dict mapping shape labels to (x, y, area) tuples for position tracking
    min_contour_area: minimum contour area (scale down for low-res, e.g. 200 for 320x240)
    """
    h, w = image.shape[:2]
    max_area = 0.9 * h * w  # treat anything this big as the outer frame

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Smooth small noise / shadows - reduced kernel size (3x3) for faster processing
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    # Let Otsu pick the threshold instead of hard-coding 220
    _, th = cv2.threshold(
    blur, 0, 255,
    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    contours, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected_labels = []
    shape_positions = {}  # Maps shape label to (x, y, area)
    annotated = image.copy()

    for c in contours:
        area = cv2.contourArea(c)
        # skip tiny specks and giant outer border
        if area < min_contour_area or area > max_area:
            continue

        label = classify(c, min_contour_area)
        if not label:
            continue

        # Calculate centroid
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Store position (use largest area if multiple detections of same shape)
            if label not in shape_positions or area > shape_positions[label][2]:
                shape_positions[label] = (cx, cy, area)
            
            # Only add each label once per frame to reduce noise
            if label not in detected_labels:
                detected_labels.append(label)

            cv2.drawContours(annotated, [c], -1, (0, 0, 0), 3)
            cv2.putText(annotated, label, (cx - 40, cy),
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 0), 2)

    return annotated, detected_labels, shape_positions

def identify_order(detected_shapes_frame):
    """
    detected_shapes_frame: list of labels seen this frame, e.g. ["Triangle", "Circle"]

    Debounce logic:
      - Only consider shapes that are NOT already in shape_order.
      - If we see the same candidate shape continuously for DEBOUNCE_SECONDS,
        we accept it and append to shape_order.
    """
    global shape_order, order_complete
    global current_candidate, candidate_start_time

    if order_complete:
        return shape_order

    # No shapes in this frame → we "lost" the target; reset candidate
    # Use grace period: with frame skipping, empty labels can be stale - don't reset on first empty
    if not detected_shapes_frame:
        if not hasattr(identify_order, 'empty_count'):
            identify_order.empty_count = 0
        identify_order.empty_count += 1
        if identify_order.empty_count >= 2:  # Require 2 consecutive empty to reset
            if current_candidate is not None:
                if DEBUG_DEBOUNCE:
                    print("[DEBUG] Lost sight of candidate, resetting debounce.")
            current_candidate = None
            candidate_start_time = None
        return shape_order
    identify_order.empty_count = 0  # Reset when we see shapes

    # Pick a candidate shape from this frame:
    # Prefer shapes that are not already in shape_order
    candidate = None
    for s in detected_shapes_frame:
        if s not in shape_order:
            candidate = s
            break

    # If everything we see is already in shape_order, nothing new to learn
    if candidate is None:
        return shape_order

    now = time.time()

    # New candidate shape → start (or restart) the debounce timer
    if candidate != current_candidate:
        current_candidate = candidate
        candidate_start_time = now
        if DEBUG_DEBOUNCE:
            print(f"[DEBUG] New candidate '{current_candidate}', starting debounce timer.")
        return shape_order

    # Same candidate as last frame → check how long we've seen it
    if candidate_start_time is not None:
        elapsed = now - candidate_start_time
        if elapsed >= DEBOUNCE_SECONDS:
            # Debounced: accept this shape into the order
            shape_order.append(current_candidate)
            if DEBUG_DEBOUNCE:
                print(f"[DEBUG] Debounced + accepted '{current_candidate}'. Order = {shape_order}")

            current_candidate = None
            candidate_start_time = None

            if len(shape_order) == 3:
                order_complete = True
                if DEBUG_DEBOUNCE:
                    print(f"[DEBUG] Final order locked in: {shape_order}")

    return shape_order


def run_video():
    global shape_order, order_complete
    global current_candidate, candidate_start_time

    # Resolution: low-res reduces processing load and stream delay
    if LOW_RES_STREAM:
        CAM_W, CAM_H = 320, 240
        min_contour_area = 200  # scale down for smaller image
        print("[INFO] Using low-res 320x240 for lower latency")
    else:
        CAM_W, CAM_H = 640, 480
        min_contour_area = 800
    if PROCESS_LATEST_ONLY:
        print("[INFO] Process-latest-only enabled (skip backlog for lower delay)")
    if PROCESS_EVERY_N_FRAMES > 1:
        print(f"[INFO] Processing every {PROCESS_EVERY_N_FRAMES} frames to reduce CPU load")

    # Initialize camera - Using ArduCam IMX219 via picamera2/libcamera
    camera = None
    use_picamera = False
    
    if PICAMERA2_AVAILABLE:
        try:
            camera = Picamera2()
            video_config = camera.create_video_configuration(
                main={"size": (CAM_W, CAM_H), "format": "RGB888"}
            )
            camera.configure(video_config)
            camera.start()
            use_picamera = True
            print(f"[INFO] ArduCam IMX219 initialized via picamera2/libcamera ({CAM_W}x{CAM_H})")
        except Exception as e:
            print(f"[ERROR] Failed to initialize ArduCam via picamera2: {e}")
            print("[INFO] Falling back to OpenCV VideoCapture")
            camera = None
    
    # Fallback to OpenCV VideoCapture if picamera2 failed
    if not use_picamera:
        print("[INFO] Trying OpenCV VideoCapture as fallback...")
        for idx in [0, 10, 11, 12, 13, 14, 15, 16, 18, 20, 21, 22, 23, 31]:
            test_camera = cv2.VideoCapture(idx)
            if test_camera.isOpened():
                ret, _ = test_camera.read()
                if ret:
                    camera = test_camera
                    print(f"[INFO] Camera found on video index {idx}")
                    break
                else:
                    test_camera.release()
            else:
                test_camera.release()
        
        if camera is None:
            print("Error: could not open camera.")
            sys.exit(1)
        
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

    # Initialize ToF sensor
    tof = TOFSensor(sensor_type='VL53L0X')  # Change to 'VL53L1X' or 'simulated' as needed
    if not tof.is_connected():
        print("[INFO] ToF sensor not connected, distance estimation will be less accurate")

    # Initialize flight controller and navigator
    fc = FlightController()
    navigator = ShapeNavigator(fc, camera_width=CAM_W, camera_height=CAM_H, tof_sensor=tof)
    
    # Initialize remote control interface
    remote_control = RemoteControl(port=8888)
    
    # Define callbacks that can access local variables
    def emergency_stop_handler():
        nonlocal navigation_active
        navigator.emergency_stop_navigation()
        navigation_active = False
    
    def status_request_handler():
        return {
            'navigation_active': navigation_active,
            'order_complete': order_complete,
            'shape_order': shape_order,
            'nav_status': navigator.get_status() if navigation_active else None
        }
    
    def reset_order_handler():
        nonlocal navigation_active, navigation_started
        global shape_order, order_complete, current_candidate, candidate_start_time
        shape_order = []
        order_complete = False
        current_candidate = None
        candidate_start_time = None
        navigation_active = False
        navigation_started = False
        navigator.reset()
        print("[INFO] Reset requested (remote), clearing shape_order and debounce state.")
    
    remote_control.set_emergency_stop_callback(emergency_stop_handler)
    remote_control.set_status_request_callback(status_request_handler)
    remote_control.set_reset_order_callback(reset_order_handler)
    remote_control.start_server()
    print("[INFO] Remote control server started on port 8888")
    print("[INFO] Connect with: telnet <pi_ip> 8888 or use remote_control.py client")
    
    # Start camera streaming server (optional - disable to reduce CPU/network overhead)
    stream_server = None
    if ENABLE_HTTP_STREAM:
        stream_server = start_streaming_server(port=8080)
    else:
        print("[INFO] HTTP streaming disabled (ENABLE_HTTP_STREAM=False) - using debug output only")
    
    # Start UDP streaming (optional - lower latency than HTTP)
    udp_stream = None
    if ENABLE_UDP_STREAM:
        udp_stream = threading.Thread(
            target=udp_stream_thread,
            args=(8081, shutdown_event),
            kwargs={"target_ip": UDP_TARGET_IP},
            daemon=True
        )
        udp_stream.start()
    else:
        print("[INFO] UDP streaming disabled (ENABLE_UDP_STREAM=False)")
    
    # Try to connect to flight controller (Flywoo GOKU GN745 V3 AIO) unless in vision-only mode
    fc_connected = False
    if VISION_ONLY:
        print("[INFO] Vision-only mode: skipping flight controller connection")
    else:
        try:
            # Try common Raspberry Pi serial ports
            # /dev/ttyAMA0 = GPIO UART (most common for Pi Zero)
            # /dev/ttyUSB0 = USB-to-serial adapter
            # /dev/ttyS0 = mini UART (may need configuration)
            print("[INFO] Attempting to connect to Flywoo GOKU GN745 V3 AIO flight controller...")
            for port in ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyS0']:
                if fc.connect_to_fc(port=port):
                    fc_connected = True
                    break
        except Exception as e:
            print(f"[INFO] Flight controller not connected: {e}")
            print("[INFO] Running in simulation mode (no flight commands will be sent)")
            print("[INFO] To connect:")
            print("  1. Wire Pi UART (GPIO 14 TX, GPIO 15 RX) to FC UART")
            print("  2. Configure ArduPilot SERIALx_PROTOCOL = 2 (MAVLink)")
            print("  3. Ensure correct baud rate (57600)")

    navigation_active = False
    navigation_started = False

    # Clear queues and reset shutdown event
    shutdown_event.clear()
    reset_from_stdin_event.clear()
    stop_from_stdin_event.clear()
    while not raw_frame_queue.empty():
        try:
            raw_frame_queue.get_nowait()
        except:
            break
    while not processed_frame_queue.empty():
        try:
            processed_frame_queue.get_nowait()
        except:
            break

    # Start frame capture thread
    capture_thread = threading.Thread(
        target=frame_capture_thread,
        args=(camera, use_picamera, raw_frame_queue, shutdown_event),
        daemon=True
    )
    capture_thread.start()
    print("[INFO] Frame capture thread started")

    # Start frame processing thread
    processing_thread = threading.Thread(
        target=frame_processing_thread,
        args=(raw_frame_queue, processed_frame_queue, tof, shutdown_event),
        kwargs={"min_contour_area": min_contour_area},
        daemon=True
    )
    processing_thread.start()
    print("[INFO] Frame processing thread started")
    print("[INFO] Using threaded processing to reduce freezing during gate detection")

    # When running in a terminal (e.g. SSH), type "r"/"reset" or "s"/"stop" + Enter
    stdin_listener = None
    if sys.stdin.isatty():
        stdin_listener = threading.Thread(
            target=stdin_listener_thread,
            args=(shutdown_event, reset_from_stdin_event, stop_from_stdin_event),
            daemon=True
        )
        stdin_listener.start()
        print("[INFO] Terminal input: 'r' or 'reset' = reset order; 's' or 'stop' = emergency stop")
    print("[INFO] Video mode: press 'q' or ESC to quit, 'r' to reset order, 'n' to start navigation manually")
    print("[INFO] Headless (no terminal): send 'reset' to port 8888 to reset order")

    while True:
        # Get processed frame from queue
        result = None
        if PROCESS_LATEST_ONLY:
            # Drain queue and use the latest result only (reduces displayed delay)
            while True:
                try:
                    result = processed_frame_queue.get_nowait()
                except Empty:
                    break
        if result is None:
            try:
                result = processed_frame_queue.get(timeout=0.05)
            except Empty:
                time.sleep(0.005)
                continue
        annotated = result['frame']
        frame_labels = result['labels']
        shape_positions = result['positions']
        distance = result['distance']

        # Learning phase: Update global shape order based on this frame
        if not navigation_active:
            identify_order(frame_labels)
            
            # Check if order is complete and start navigation
            if order_complete and not navigation_started:
                print("[INFO] Shape order complete! Starting navigation...")
                print(f"[INFO] Order: {shape_order}")
                ok = navigator.start_navigation(shape_order)
                if ok:
                    navigation_active = True
                    navigation_started = True
                    print("[INFO] Navigation started successfully - gate detection active")
                else:
                    print("[INFO] Navigation start failed, continuing in learning mode")

        # Navigation phase: Update navigation based on detections
        if navigation_active:
            navigator.update_navigation(frame_labels, shape_positions, distance)
            nav_status = navigator.get_status()
            
            # Check if navigation is complete or errored
            if nav_status['state'] == navigator.STATE_COMPLETE:
                navigation_active = False
                print("[INFO] Navigation complete! Mission finished.")
            elif nav_status['state'] == navigator.STATE_ERROR:
                navigation_active = False
                print("[INFO] Navigation error! Safety procedures executed.")

        # Draw current shape order on the frame
        order_text = "Order: " + " -> ".join(shape_order) if shape_order else "Order: (learning...)"
        cv2.putText(annotated, order_text, (20, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)

        if order_complete:
            cv2.putText(annotated, "ORDER LOCKED", (20, 60),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 128, 0), 2)

        # Draw navigation status and ToF distance
        if navigation_active:
            nav_status = navigator.get_status()
            status_text = f"NAV: {nav_status['state']} | Target: {nav_status['current_target']} ({nav_status['progress']})"
            cv2.putText(annotated, status_text, (20, 90),
                        cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 0, 0), 2)
        
        # Display ToF distance
        if distance is not None:
            dist_text = f"Distance: {distance:.2f}m"
            cv2.putText(annotated, dist_text, (20, 120),
                        cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 255), 2)
            
            # Highlight target shape if detected
        if navigation_active:
            nav_status = navigator.get_status()
            if nav_status['current_target'] in frame_labels:
                if nav_status['current_target'] in shape_positions:
                    x, y, _ = shape_positions[nav_status['current_target']]
                    cv2.circle(annotated, (x, y), 30, (0, 255, 0), 3)
                    cv2.putText(annotated, "TARGET", (x - 30, y - 40),
                                cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 2)
        
        # Display remote control status
        cv2.putText(annotated, "Remote: Port 8888", (20, annotated.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

        # Update latest frame for streaming (HTTP or UDP)
        if ENABLE_HTTP_STREAM or ENABLE_UDP_STREAM:
            with frame_lock:
                global latest_frame
                latest_frame = annotated.copy()

        # Display window or print status (headless mode)
        if HEADLESS_MODE:
            # No display available - run in headless mode
            key = -1
            # Print status to console instead (throttled to avoid spam)
            if not hasattr(run_video, 'last_print_time'):
                run_video.last_print_time = 0
            if time.time() - run_video.last_print_time > 2.0:  # Print every 2 seconds
                if len(frame_labels) > 0:
                    print(f"[FRAME] Detected: {frame_labels}, Order: {shape_order}")
                run_video.last_print_time = time.time()
        else:
            # Try to display window
            try:
                cv2.imshow("ShaperDrone - Video", annotated)
                key = cv2.waitKey(1) & 0xFF
            except Exception:
                # Fallback if display fails
                key = -1
                print("[WARN] Display window failed, continuing in headless mode")

        if key == 27 or key == ord('q'):  # ESC or 'q'
            break

        # reset order (from OpenCV key 'r' or from terminal stdin when headless)
        if key == ord('r') or reset_from_stdin_event.is_set():
            if reset_from_stdin_event.is_set():
                reset_from_stdin_event.clear()
            print("[INFO] Reset requested, clearing shape_order and debounce state.")
            shape_order = []
            order_complete = False
            current_candidate = None
            candidate_start_time = None
            navigation_active = False
            navigation_started = False
            navigator.reset()

        # Manual navigation start
        if key == ord('n'):
            if order_complete and not navigation_active:
                print("[INFO] Manual navigation start requested...")
                if navigator.start_navigation(shape_order):
                    navigation_active = True
                    navigation_started = True
            elif not order_complete:
                print("[INFO] Cannot start navigation: shape order not complete yet")
        
        # Emergency stop (from OpenCV key 's' or from terminal stdin)
        if key == ord('s') or stop_from_stdin_event.is_set():
            if stop_from_stdin_event.is_set():
                stop_from_stdin_event.clear()
            if navigation_active:
                print("[INFO] Emergency stop requested!")
                navigator.emergency_stop_navigation()
                navigation_active = False
        
    # Cleanup: Signal threads to stop
    print("[INFO] Shutting down threads...")
    shutdown_event.set()
    
    # Wait for threads to finish (with timeout)
    capture_thread.join(timeout=1.0)
    processing_thread.join(timeout=1.0)
    
    # Cleanup
    remote_control.stop_server()
    if fc_connected:
        fc.close()
    if use_picamera:
        camera.stop()
        camera.close()
    else:
        camera.release()
    tof.close()
    if not HEADLESS_MODE:
        try:
            cv2.destroyAllWindows()
        except:
            pass  # Display cleanup failed
    


def main():
    # If user provides an image path: image mode
    if len(sys.argv) >= 2:
        print("Error: This program is not intended to work with static images.")
        return
    else:
        # No args → default to video mode
        run_video()


if __name__ == "__main__":
    main()