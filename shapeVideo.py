# Transitioning from static images to video shape detection

import cv2
import time
import numpy as np
# from pymavlink import mavutil  # commented out for now
import sys


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
    annotated = image.copy()

    for c in contours:
        area = cv2.contourArea(c)
        # skip tiny specks and giant outer border
        if area < 800 or area > max_area:
            continue

        label = classify(c)
        if not label:
            continue

        # Only add each label once per frame to reduce noise
        if label not in detected_labels:
            detected_labels.append(label)

        cv2.drawContours(annotated, [c], -1, (0, 0, 0), 3)

        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.putText(annotated, label, (cx - 40, cy),
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 0), 2)

    return annotated, detected_labels

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

    cap = cv2.VideoCapture(0) # not forcing V4L2 backend when running on Windows, 0 = default -> change if needed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    if not cap.isOpened():
        print("Error: could not open camera.")
        sys.exit(1)

    print("[INFO] Video mode: press 'q' or ESC to quit, 'r' to reset order.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: could not read frame from camera.")
            break

        annotated, frame_labels = process_frame(frame)

        # Update global shape order based on this frame
        identify_order(frame_labels)

        # Draw current shape order on the frame
        order_text = "Order: " + " -> ".join(shape_order) if shape_order else "Order: (learning...)"
        cv2.putText(annotated, order_text, (20, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)

        if order_complete:
            cv2.putText(annotated, "ORDER LOCKED", (20, 60),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 128, 0), 2)

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
        
        
    cap.release()
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