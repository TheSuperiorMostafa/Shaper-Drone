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
DEBOUNCE_SECONDS = 1.5  # require ~1.5s of stable viewing to add a shape to the array

STATE_ORDER = "ORDER"
STATE_IN_FLIGHT = "IN_FLIGHT"
state = STATE_ORDER

POST_ORDER_HOLD_SECONDS = 5.0
order_locked_time = None  # when order_complete first became True

last_order_polarity = None

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

def threshold_auto_polarity(gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, th_bin = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, th_inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # light cleanup on both
    kernel = np.ones((5,5), np.uint8)
    th_bin = cv2.morphologyEx(th_bin, cv2.MORPH_OPEN, kernel, iterations=1)
    th_inv = cv2.morphologyEx(th_inv, cv2.MORPH_OPEN, kernel, iterations=1)

    return th_bin, th_inv


def process_frame(image):
    """
    Takes a BGR image, returns:
      - annotated_image
      - list of labels detected in this frame (e.g. ["Triangle", "Circle"])
    """

    global last_order_polarity
    
    h, w = image.shape[:2]
    max_area = 0.9 * h * w  # treat anything this big as the outer frame

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # --- pick best threshold polarity (BIN vs INV) ---
    th_bin, th_inv = threshold_auto_polarity(gray)

    def score_mask(th):
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < 800 or area > max_area:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 0:
                continue
            circ = 4.0 * np.pi * area / (perim * perim)
            # Prefer compact blobs; cap circularity at 1
            best = max(best, area * min(circ, 1.0))
        return best

    score_bin = score_mask(th_bin)
    score_inv = score_mask(th_inv)
  
    th = th_inv if score_inv > score_bin else th_bin
    cv2.imshow("th_order", th)

    pol = "INV" if score_inv > score_bin else "BIN"
    if pol != last_order_polarity:
        print(f"[ORDER] Polarity switched -> {pol} (bin={score_bin:.1f}, inv={score_inv:.1f})")
        last_order_polarity = pol

    # --- contours from chosen mask ---
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_labels = []
    annotated = image.copy()

    for c in contours:
        area = cv2.contourArea(c)
        if area < 800 or area > max_area:
            continue

        label = classify(c)
        if not label:
            continue

        if label not in detected_labels:
            detected_labels.append(label)

        cv2.drawContours(annotated, [c], -1, (0, 0, 0), 3)

        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.putText(
                annotated, label, (cx - 40, cy),
                cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 0), 2
            )

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
    global state, order_locked_time

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("Error: could not open camera.")
        sys.exit(1)

    print("[INFO] Currently detecting filled-in shape order | ESC/q quit | r reset | n next (force IN_FLIGHT)")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: could not read frame from camera.")
            break

        key = cv2.waitKey(1) & 0xFF

        # Quit
        if key == 27 or key == ord('q'):
            break

        # Force next
        if key == ord('n'):
            print("[INFO] Forced NEXT: entering IN_FLIGHT.")
            state = STATE_IN_FLIGHT

        # Reset
        if key == ord('r'):
            print("[INFO] Reset requested.")
            shape_order = []
            order_complete = False
            current_candidate = None
            candidate_start_time = None
            order_locked_time = None
            state = STATE_ORDER
            continue

        # -------------------- ORDER MODE --------------------
        if state == STATE_ORDER:
            annotated, frame_labels = process_frame(frame)
            identify_order(frame_labels)

            # If we just reached order_complete, start the 5s timer
            now = time.time()
            if order_complete and order_locked_time is None:
                order_locked_time = now
                print("[INFO] Order locked. Starting 5s hold timer...")

            # If order_complete is not true anymore (shouldn't happen), reset timer
            if not order_complete:
                order_locked_time = None

            # Draw HUD
            order_text = "Order: " + " -> ".join(shape_order) if shape_order else "Order: (learning...)"
            cv2.putText(annotated, order_text, (20, 30),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)

            if order_complete:
                elapsed = now - order_locked_time if order_locked_time else 0.0
                cv2.putText(annotated,
                            f"ORDER LOCKED - holding {elapsed:0.1f}/{POST_ORDER_HOLD_SECONDS:.1f}s (press 'n' to skip)",
                            (20, 60), cv2.FONT_HERSHEY_DUPLEX, 0.65, (0, 200, 0), 2)

                if elapsed >= POST_ORDER_HOLD_SECONDS:
                    print("[INFO] 5s hold complete -> entering IN_FLIGHT.")
                    state = STATE_IN_FLIGHT

            cv2.imshow("ShaperDrone - Video", annotated)

        # -------------------- IN-FLIGHT MODE (gate inner hole) --------------------
        elif state == STATE_IN_FLIGHT:
            det, debug = detect_gate_inner(frame, use_clahe=True, draw_debug=True)

            cv2.putText(debug, "IN FLIGHT (inner-hole gate code)", (20, 60),
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 200, 0), 2)

            # Optional: show order on screen
            if shape_order:
                cv2.putText(debug, "Order: " + " -> ".join(shape_order), (20, 90),
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("ShaperDrone - Video", debug)

    cap.release()
    cv2.destroyAllWindows()
    
def preprocess_for_contours(frame, use_clahe=False):
    """Return binary mask for contour extraction."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if use_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Otsu threshold; keep INV if your gate/frame is darker than background.
    _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)


    # Optional cleanup (helps specks)
    kernel = np.ones((5,5), np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)


    return th

def contour_centroid(contour):
    """Return (cx, cy) or None if degenerate."""
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    return (M["m10"] / M["m00"], M["m01"] / M["m00"])

def pick_best_gate_hole(contours, hierarchy, frame_shape):
    """
    Find best (outer_contour, inner_hole_contour) using hierarchy.
    Gate = contour that has a child (hole). We classify/centroid the child.
    """
    if hierarchy is None or len(contours) == 0:
        return None, None

    H, W = frame_shape[:2]
    hier = hierarchy[0]

    best_parent = None
    best_child = None
    best_score = -1e18

    for parent_idx, parent in enumerate(contours):
        parent_area = cv2.contourArea(parent)
        if parent_area < 1000 or parent_area > 0.98 * H * W:
            continue

        # Walk ALL children of this parent (first_child + siblings)
        child_idx = hier[parent_idx][2]
        while child_idx != -1:
            child = contours[child_idx]
            child_area = cv2.contourArea(child)

            if child_area >= 800:
                hole_ratio = child_area / float(parent_area)
                if 0.15 <= hole_ratio <= 0.95:
                    cxy = contour_centroid(child)
                    if cxy is not None:
                        cx, cy = cxy
                        center_dist = ((cx - W/2)**2 + (cy - H/2)**2) ** 0.5
                        score = child_area - (0.7 * center_dist)

                        if score > best_score:
                            best_score = score
                            best_parent = parent
                            best_child = child

            child_idx = hier[child_idx][0]  # next sibling

    return best_parent, best_child

def classify_opening(contour):
    """
    Classify the OPENING (inner contour).
    Use your existing classify logic, but it should be tolerant to “hole edges”.
    """
    area = cv2.contourArea(contour)
    if area < 800:
        return None

    perim = cv2.arcLength(contour, True)
    if perim == 0:
        return None

    circularity = 4.0 * np.pi * area / (perim * perim)

    eps = 0.01 * perim
    approx = cv2.approxPolyDP(contour, eps, True)
    n = len(approx)

    # Circle gate opening
    if n >= 7 and circularity > 0.80:
        return "Circle"

    if n == 3:
        return "Triangle"
    if n == 4:
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        return "Square" if 0.9 < ar < 1.1 else "Rectangle"
    if n == 5:
        return "Pentagon"
    if n == 6:
        return "Hexagon"

    return None

def detect_gate_inner(frame, use_clahe=False, draw_debug=True):
    """
    Returns:
      detection dict or None

    detection dict fields:
      - label: "Circle"/"Square"/...
      - centroid_px: (cx, cy) in pixels
      - error_norm: (ex, ey) where ex,ey in [-1,1] (relative to image center)
      - hole_area: area of inner opening contour
      - outer_contour / inner_contour: contours (optional for navigation/debug)
      - debug_frame: annotated frame if draw_debug=True
    """
    H, W = frame.shape[:2]
    th = preprocess_for_contours(frame, use_clahe=use_clahe)

    # ALWAYS show threshold for debugging
    cv2.imshow("gate_th", th)

    contours, hierarchy = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    outer, inner = pick_best_gate_hole(contours, hierarchy, frame.shape)

    debug = frame.copy()

    if inner is None:
        if draw_debug:
            cv2.putText(debug, "NO GATE DETECTED", (20, 30),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)
            return None, debug
        return None, debug

    label = classify_opening(inner)
    cxy = contour_centroid(inner)
    if label is None or cxy is None:
        if draw_debug:
            cv2.putText(debug, "FOUND HOLE, FAILED CLASSIFY", (20, 30),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 2)
            return None, debug
        return None, debug

    cx, cy = cxy
    ex = (cx - W/2) / (W/2)
    ey = (cy - H/2) / (H/2)

    det = {
        "label": label,
        "centroid_px": (cx, cy),
        "error_norm": (ex, ey),
        "hole_area": cv2.contourArea(inner),
        "outer_contour": outer,
        "inner_contour": inner,
    }

    if draw_debug:
        cv2.drawContours(debug, [outer], -1, (0, 255, 0), 2)
        cv2.drawContours(debug, [inner], -1, (255, 0, 0), 2)
        cv2.circle(debug, (int(cx), int(cy)), 4, (0, 0, 255), -1)
        cv2.putText(debug, f"{label}  ex={ex:.2f} ey={ey:.2f}",
                    (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
        return det, debug

    return det


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