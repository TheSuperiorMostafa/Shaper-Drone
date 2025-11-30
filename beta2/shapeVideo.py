# Transitioning from static images to video shape detection

import cv2
import time
import numpy as np
import sys
from navigation import FlightController
from shape_navigator import ShapeNavigator
from tof_sensor import TOFSensor
from remote_control import RemoteControl

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

def classify(contour):
    area = cv2.contourArea(contour)

    if area < 800:
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


def process_frame(image):
    """
    Takes a BGR image, returns:
      - annotated_image
      - list of labels detected in this frame (e.g. ["Triangle", "Circle"])
      - dict mapping shape labels to (x, y, area) tuples for position tracking
    """
    h, w = image.shape[:2]
    max_area = 0.9 * h * w  # treat anything this big as the outer frame

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Smooth small noise / shadows
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

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
        if area < 800 or area > max_area:
            continue

        label = classify(c)
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
    if not detected_shapes_frame:
        if current_candidate is not None:
            print("[DEBUG] Lost sight of candidate, resetting debounce.")
        current_candidate = None
        candidate_start_time = None
        return shape_order

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
        print(f"[DEBUG] New candidate '{current_candidate}', starting debounce timer.")
        return shape_order

    # Same candidate as last frame → check how long we've seen it
    if candidate_start_time is not None:
        elapsed = now - candidate_start_time
        if elapsed >= DEBOUNCE_SECONDS:
            # Debounced: accept this shape into the order
            shape_order.append(current_candidate)
            print(f"[DEBUG] Debounced + accepted '{current_candidate}'. Order = {shape_order}")

            current_candidate = None
            candidate_start_time = None

            if len(shape_order) == 3:
                order_complete = True
                print(f"[DEBUG] Final order locked in: {shape_order}")

    return shape_order


def run_video():
    global shape_order, order_complete
    global current_candidate, candidate_start_time

    # Initialize camera (Raspberry Pi camera or USB fallback)
    camera = None
    use_picamera = False
    
    if PICAMERA2_AVAILABLE:
        try:
            camera = Picamera2()
            # Configure camera for video
            video_config = camera.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
            camera.configure(video_config)
            camera.start()
            use_picamera = True
            print("[INFO] Using Raspberry Pi camera module (picamera2)")
        except Exception as e:
            print(f"[INFO] Failed to initialize Pi camera: {e}")
            print("[INFO] Falling back to USB camera")
            camera = None
    
    if not use_picamera:
        # Fallback to USB camera
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not camera.isOpened():
            print("Error: could not open camera.")
            sys.exit(1)
        print("[INFO] Using USB camera (OpenCV VideoCapture)")

    # Initialize ToF sensor
    tof = TOFSensor(sensor_type='VL53L0X')  # Change to 'VL53L1X' or 'simulated' as needed
    if not tof.is_connected():
        print("[INFO] ToF sensor not connected, distance estimation will be less accurate")

    # Initialize flight controller and navigator
    fc = FlightController()
    navigator = ShapeNavigator(fc, camera_width=640, camera_height=480, tof_sensor=tof)
    
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
    
    remote_control.set_emergency_stop_callback(emergency_stop_handler)
    remote_control.set_status_request_callback(status_request_handler)
    remote_control.start_server()
    print("[INFO] Remote control server started on port 8888")
    print("[INFO] Connect with: telnet <pi_ip> 8888 or use remote_control.py client")
    
    # Try to connect to flight controller (Flywoo GOKU GN745 V3 AIO)
    # Connect Pi UART (GPIO 14/15) to any FC UART port configured for MAVLink
    fc_connected = False
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

    print("[INFO] Video mode: press 'q' or ESC to quit, 'r' to reset order, 'n' to start navigation manually")

    while True:
        # Read frame from camera
        if use_picamera:
            frame = camera.capture_array()
            # picamera2 returns RGB, convert to BGR for OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = camera.read()
            if not ret:
                print("Error: could not read frame from camera.")
                break

        # Read ToF sensor distance
        distance = tof.read_distance()
        
        annotated, frame_labels, shape_positions = process_frame(frame)

        # Learning phase: Update global shape order based on this frame
        if not navigation_active:
            identify_order(frame_labels)
            
            # Check if order is complete and start navigation
            if order_complete and not navigation_started:
                print("[INFO] Shape order complete! Starting navigation...")
                if navigator.start_navigation(shape_order):
                    navigation_active = True
                    navigation_started = True
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

        cv2.imshow("ShaperDrone - Video", annotated)

        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord('q'):  # ESC or 'q'
            break

        # reset order
        if key == ord('r'):
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
        
        # Emergency stop
        if key == ord('s'):
            if navigation_active:
                print("[INFO] Emergency stop requested!")
                navigator.emergency_stop_navigation()
                navigation_active = False
        
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
    cv2.destroyAllWindows()
    


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