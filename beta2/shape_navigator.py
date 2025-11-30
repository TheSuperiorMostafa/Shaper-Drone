"""
Shape navigation module - manages autonomous navigation through a sequence of shapes.
Handles state machine, visual servoing, and shape reach detection.
"""

import time
import math
from navigation import FlightController

class ShapeNavigator:
    """Manages navigation through a sequence of detected shapes."""
    
    # Navigation states
    STATE_IDLE = "idle"
    STATE_TAKEOFF = "takeoff"
    STATE_SEARCHING = "searching"
    STATE_APPROACHING = "approaching"
    STATE_REACHED = "reached"
    STATE_FLYING_THROUGH = "flying_through"  # New state for gate navigation
    STATE_COMPLETE = "complete"
    STATE_ERROR = "error"
    
    def __init__(self, flight_controller, camera_width=640, camera_height=480, tof_sensor=None):
        """
        Initialize shape navigator.
        
        Args:
            flight_controller: FlightController instance
            camera_width: Camera frame width in pixels
            camera_height: Camera frame height in pixels
            tof_sensor: TOFSensor instance for distance measurements (optional)
        """
        self.fc = flight_controller
        self.tof = tof_sensor
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_center_x = camera_width / 2.0
        self.camera_center_y = camera_height / 2.0
        
        # Navigation state
        self.state = self.STATE_IDLE
        self.shape_order = []
        self.current_target_index = 0
        self.current_target_shape = None
        
        # Visual servoing parameters
        self.max_velocity = 1.0  # m/s maximum velocity
        self.approach_distance = 2.0  # meters - consider shape "reached" at this distance
        self.search_timeout = 30.0  # seconds to search for shape before timeout
        self.max_navigation_time = 300.0  # maximum total navigation time (5 minutes)
        
        # Gate navigation parameters
        self.gate_pass_duration = 2.0  # seconds to continue forward after reaching gate
        self.gate_pass_velocity = 0.5  # m/s forward velocity when passing through gate
        
        # Safety parameters
        self.emergency_stop = False
        self.navigation_start_time = None
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # Tracking
        self.target_last_seen = None
        self.search_start_time = None
        self.shape_reached_time = None
        self.reach_confirm_duration = 2.0  # seconds to confirm shape is reached
        self.gate_pass_start_time = None  # Time when started flying through gate
        
        # Visual servoing gains
        self.kp_x = 0.5  # proportional gain for horizontal movement
        self.kp_y = 0.5  # proportional gain for forward/back movement
        self.kp_z = 0.3  # proportional gain for vertical movement
        
        # Shape size tracking (for distance estimation when ToF not available)
        self.target_shape_size = None
        self.reference_shape_size = 1000  # pixels - reference size at approach_distance
        
        # ToF distance tracking
        self.current_distance = None
        self.distance_history = []  # For filtering noisy readings
        self.distance_history_size = 5
        
    def start_navigation(self, shape_order):
        """
        Initialize navigation with learned shape sequence.
        
        Args:
            shape_order: List of shape names in order, e.g. ["Triangle", "Square", "Circle"]
        """
        if len(shape_order) == 0:
            print("[NAV] Error: Cannot start navigation with empty shape order")
            self.state = self.STATE_ERROR
            return False
        
        # Safety check: ensure flight controller is connected
        if not self.fc.connected:
            print("[NAV] Warning: Flight controller not connected. Navigation will run in simulation mode.")
        
        self.shape_order = shape_order.copy()
        self.current_target_index = 0
        self.current_target_shape = self.shape_order[0]
        self.state = self.STATE_TAKEOFF  # Start with takeoff
        self.search_start_time = time.time()
        self.navigation_start_time = time.time()
        self.target_last_seen = None
        self.emergency_stop = False
        self.consecutive_errors = 0
        
        print(f"[NAV] Navigation started. Target sequence: {' -> '.join(self.shape_order)}")
        
        # Ensure we're in GUIDED mode
        if self.fc.connected:
            if not self.fc.set_guided_mode():
                print("[NAV] Warning: Could not set GUIDED mode")
                return False
            
            # Start takeoff sequence
            print("[NAV] Initiating automatic takeoff...")
            if self.fc.takeoff(altitude=1.5):  # Takeoff to 1.5m for indoor flight
                print("[NAV] Takeoff command sent, waiting for takeoff completion...")
            else:
                print("[NAV] Warning: Takeoff command failed")
                return False
        else:
            # Simulation mode - skip takeoff
            print("[NAV] Simulation mode: Skipping takeoff, starting search")
            self.state = self.STATE_SEARCHING
        
        return True
    
    def update_navigation(self, detected_shapes, shape_positions, distance=None):
        """
        Update navigation based on current frame detection.
        
        Args:
            detected_shapes: List of shape labels detected in current frame
            shape_positions: Dict mapping shape labels to (x, y, area) tuples
            distance: Current distance from ToF sensor in meters (optional)
        
        Returns:
            str: Current navigation state
        """
        # Update distance from ToF sensor
        if distance is not None:
            self.current_distance = distance
            # Add to history for filtering
            self.distance_history.append(distance)
            if len(self.distance_history) > self.distance_history_size:
                self.distance_history.pop(0)
        elif self.tof and self.tof.is_connected():
            # Try to read from ToF sensor if not provided
            self.current_distance = self.tof.read_distance()
            if self.current_distance:
                self.distance_history.append(self.current_distance)
                if len(self.distance_history) > self.distance_history_size:
                    self.distance_history.pop(0)
        if self.state == self.STATE_IDLE or self.state == self.STATE_COMPLETE:
            return self.state
        
        if self.state == self.STATE_ERROR:
            return self.state
        
        # Handle takeoff state
        if self.state == self.STATE_TAKEOFF:
            if self.fc.connected:
                # Check if we've reached takeoff altitude
                self.fc.update_vehicle_state()
                current_alt = self.fc.vehicle_state['position']['alt']
                if current_alt >= 1.2:  # Reached at least 1.2m (close enough to 1.5m target)
                    print(f"[NAV] Takeoff complete! Altitude: {current_alt:.2f}m")
                    self.state = self.STATE_SEARCHING
                    self.search_start_time = time.time()
                    print(f"[NAV] Searching for first target: {self.current_target_shape}")
                else:
                    # Still taking off, hover in place
                    self.fc.send_velocity_command(0, 0, 0)
            else:
                # Simulation mode - skip takeoff wait
                self.state = self.STATE_SEARCHING
                self.search_start_time = time.time()
            return self.state
        
        # Handle gate flying through state
        if self.state == self.STATE_FLYING_THROUGH:
            if self.gate_pass_start_time is None:
                self.gate_pass_start_time = time.time()
            
            # Continue forward through the gate
            elapsed = time.time() - self.gate_pass_start_time
            if elapsed < self.gate_pass_duration:
                # Keep flying forward through gate
                self.fc.send_velocity_command(self.gate_pass_velocity, 0, 0)
                return self.state
            else:
                # Finished passing through gate, move to next shape
                print(f"[NAV] Successfully flew through gate with shape: {self.current_target_shape}")
                self.gate_pass_start_time = None
                self.move_to_next_shape()
                return self.state
        
        # Emergency stop check
        if self.emergency_stop:
            self.fc.send_velocity_command(0, 0, 0)
            self.state = self.STATE_ERROR
            return self.state
        
        # Check for overall navigation timeout
        if self.navigation_start_time:
            elapsed_total = time.time() - self.navigation_start_time
            if elapsed_total > self.max_navigation_time:
                print(f"[NAV] Safety timeout: Maximum navigation time ({self.max_navigation_time}s) exceeded")
                self.handle_navigation_error("Maximum navigation time exceeded")
                return self.state
        
        # Check for timeout on current target
        if self.state == self.STATE_SEARCHING:
            if time.time() - self.search_start_time > self.search_timeout:
                print(f"[NAV] Timeout: Could not find target shape '{self.current_target_shape}'")
                self.consecutive_errors += 1
                
                if self.consecutive_errors >= self.max_consecutive_errors:
                    self.handle_navigation_error("Too many consecutive timeouts")
                else:
                    # Try to skip to next shape
                    print(f"[NAV] Attempting to skip to next shape ({self.consecutive_errors}/{self.max_consecutive_errors} errors)")
                    self.move_to_next_shape()
                
                return self.state
        
        # Check if target shape is detected
        target_detected = self.current_target_shape in detected_shapes
        
        if target_detected:
            self.target_last_seen = time.time()
            
            # Get target shape position
            if self.current_target_shape in shape_positions:
                x, y, area = shape_positions[self.current_target_shape]
                
                # Update target shape size (use largest detection if multiple)
                if self.target_shape_size is None or area > self.target_shape_size:
                    self.target_shape_size = area
                
                # Check if we've reached the shape (use ToF distance if available)
                if self.is_shape_reached(area):
                    if self.state != self.STATE_REACHED:
                        self.state = self.STATE_REACHED
                        self.shape_reached_time = time.time()
                        print(f"[NAV] Reached gate with shape: {self.current_target_shape}")
                        if self.current_distance:
                            print(f"[NAV] Distance at gate: {self.current_distance:.2f}m")
                    
                    # Confirm we're at the gate for a duration, then fly through it
                    if time.time() - self.shape_reached_time >= self.reach_confirm_duration:
                        # Start flying through the gate
                        self.state = self.STATE_FLYING_THROUGH
                        self.gate_pass_start_time = None  # Will be set in next update
                        print(f"[NAV] Flying through gate with shape: {self.current_target_shape}")
                    
                    # Hold position while confirming
                    self.fc.send_velocity_command(0, 0, 0)
                    return self.state
                
                # Navigate toward shape using visual servoing
                if self.state == self.STATE_SEARCHING:
                    self.state = self.STATE_APPROACHING
                    print(f"[NAV] Target '{self.current_target_shape}' detected, approaching...")
                
                self.visual_servo(x, y, area)
                
            else:
                # Shape detected but no position data
                print(f"[NAV] Warning: Target shape detected but no position data")
        
        else:
            # Target shape not detected
            if self.state == self.STATE_APPROACHING:
                # Lost sight of target - continue last command briefly
                if self.target_last_seen and time.time() - self.target_last_seen < 1.0:
                    # Continue approaching for 1 second after losing sight
                    pass
                else:
                    # Lost target, return to searching
                    self.state = self.STATE_SEARCHING
                    self.fc.send_velocity_command(0, 0, 0)  # Stop
                    print(f"[NAV] Lost sight of target, searching...")
            elif self.state == self.STATE_SEARCHING:
                # Search pattern: slow rotation or hover
                self.fc.send_velocity_command(0, 0, 0)  # Hover in place
        
        return self.state
    
    def visual_servo(self, shape_x, shape_y, shape_area):
        """
        Convert camera pixel coordinates to velocity commands for visual servoing.
        Uses ToF sensor distance if available for more accurate navigation.
        
        Args:
            shape_x: X coordinate of shape center in pixels
            shape_y: Y coordinate of shape center in pixels
            shape_area: Area of shape in pixels
        """
        # Calculate offset from center
        dx = shape_x - self.camera_center_x  # positive = shape is right of center
        dy = self.camera_center_y - shape_y  # positive = shape is above center (camera coords)
        
        # Normalize to [-1, 1] range
        norm_dx = dx / self.camera_center_x
        norm_dy = dy / self.camera_center_y
        
        # Calculate velocity commands
        # In NED frame: vx = forward, vy = right, vz = down
        # If shape is right of center, move right (positive vy)
        # If shape is above center, move forward (positive vx)
        vx = self.kp_y * norm_dy * self.max_velocity  # forward/back
        vy = self.kp_x * norm_dx * self.max_velocity  # left/right
        
        # Use ToF sensor distance if available (more accurate than shape size estimation)
        if self.current_distance is not None and len(self.distance_history) > 0:
            # Use median of recent readings for stability
            filtered_distance = sorted(self.distance_history)[len(self.distance_history) // 2]
            
            # Distance-based forward/back control
            distance_error = filtered_distance - self.approach_distance
            
            if distance_error < -0.3:  # Too close (more than 30cm closer than target)
                # Move back
                vx = -0.4 * self.max_velocity
            elif distance_error > 0.5:  # Too far (more than 50cm farther than target)
                # Move forward more aggressively
                vx = max(vx, 0.6 * self.max_velocity)
            else:
                # Close to target distance, use visual servoing for fine adjustment
                # Scale forward velocity based on distance error
                vx = vx + 0.3 * distance_error * self.max_velocity
        
        # Fallback: Estimate distance from shape size if ToF not available
        elif self.target_shape_size and shape_area > 0:
            # Simple distance estimation based on area
            # When shape is at approach_distance, it should be reference_shape_size
            estimated_distance = self.approach_distance * math.sqrt(self.reference_shape_size / shape_area)
            
            # If too close, move back; if too far, move forward
            if estimated_distance < self.approach_distance * 0.8:
                # Too close, move back
                vx = -0.3 * self.max_velocity
            elif estimated_distance > self.approach_distance * 1.5:
                # Too far, move forward more aggressively
                vx = max(vx, 0.5 * self.max_velocity)
        else:
            # Default: move forward if shape is in view
            if abs(norm_dx) < 0.3 and abs(norm_dy) < 0.3:
                # Shape is roughly centered, move forward
                vx = 0.3 * self.max_velocity
        
        # Limit velocities
        vx = max(-self.max_velocity, min(self.max_velocity, vx))
        vy = max(-self.max_velocity, min(self.max_velocity, vy))
        
        # Send velocity command
        self.fc.send_velocity_command(vx, vy, 0)  # No vertical movement for now
    
    def is_shape_reached(self, shape_area):
        """
        Check if shape is close enough to be considered "reached".
        Uses ToF sensor distance if available for accurate detection.
        
        Args:
            shape_area: Area of shape in pixels
        
        Returns:
            bool: True if shape is reached
        """
        # Use ToF sensor distance if available (most accurate)
        if self.current_distance is not None:
            # Consider reached if within 30cm of target distance
            distance_tolerance = 0.3
            return abs(self.current_distance - self.approach_distance) <= distance_tolerance
        
        # Fallback: Use shape size estimation
        if not self.target_shape_size:
            # Use area-based heuristic: if shape is large enough, we're close
            return shape_area > self.reference_shape_size * 0.8
        
        # Shape is reached if it's at least as large as our reference size
        # (meaning we're at or closer than approach_distance)
        return shape_area >= self.reference_shape_size * 0.9
    
    def move_to_next_shape(self):
        """Move to next shape in sequence."""
        self.current_target_index += 1
        self.consecutive_errors = 0  # Reset error count on successful shape completion
        
        if self.current_target_index >= len(self.shape_order):
            # All shapes visited
            self.state = self.STATE_COMPLETE
            self.fc.send_velocity_command(0, 0, 0)  # Stop
            print(f"[NAV] Navigation complete! Visited all shapes: {' -> '.join(self.shape_order)}")
            
            # Safety: After completing mission, return to launch
            if self.fc.connected:
                print("[NAV] Mission complete. Returning to launch...")
                self.fc.return_to_launch()
        else:
            # Move to next shape
            self.current_target_shape = self.shape_order[self.current_target_index]
            self.state = self.STATE_SEARCHING
            self.search_start_time = time.time()
            self.target_last_seen = None
            self.target_shape_size = None  # Reset for new target
            print(f"[NAV] Moving to next target ({self.current_target_index + 1}/{len(self.shape_order)}): {self.current_target_shape}")
    
    def get_next_target(self):
        """
        Get the next target shape in sequence.
        
        Returns:
            str: Next target shape name, or None if complete
        """
        if self.current_target_index < len(self.shape_order):
            return self.shape_order[self.current_target_index]
        return None
    
    def get_status(self):
        """
        Get current navigation status.
        
        Returns:
            dict: Status information including state, current target, progress
        """
        return {
            'state': self.state,
            'current_target': self.current_target_shape,
            'target_index': self.current_target_index,
            'total_shapes': len(self.shape_order),
            'progress': f"{self.current_target_index}/{len(self.shape_order)}" if self.shape_order else "0/0"
        }
    
    def handle_navigation_error(self, error_message):
        """
        Handle navigation errors with safety measures.
        
        Args:
            error_message: Description of the error
        """
        print(f"[NAV] Navigation error: {error_message}")
        self.state = self.STATE_ERROR
        
        # Stop all movement
        self.fc.send_velocity_command(0, 0, 0)
        
        # Safety: Return to launch or land
        if self.fc.connected:
            print("[NAV] Executing safety procedure: Return to Launch")
            self.fc.return_to_launch()
    
    def emergency_stop_navigation(self):
        """
        Immediately stop navigation and execute safety procedure.
        This is the motor kill switch - immediately disarms motors.
        """
        print("[NAV] EMERGENCY STOP (MOTOR KILL SWITCH) activated!")
        self.emergency_stop = True
        self.fc.send_velocity_command(0, 0, 0)
        
        # Immediately disarm motors (kill switch)
        if self.fc.connected:
            self.fc.emergency_disarm()
            print("[NAV] Motors disarmed - drone will fall into safety net")
    
    def reset(self):
        """Reset navigator to idle state."""
        self.state = self.STATE_IDLE
        self.shape_order = []
        self.current_target_index = 0
        self.current_target_shape = None
        self.target_last_seen = None
        self.search_start_time = None
        self.shape_reached_time = None
        self.target_shape_size = None
        self.navigation_start_time = None
        self.emergency_stop = False
        self.consecutive_errors = 0
        self.gate_pass_start_time = None
        print("[NAV] Navigator reset")

