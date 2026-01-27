"""
Navigation module for ArduPilot flight controller communication via MAVLink.
Handles connection, flight mode control, and command sending.

Compatible with:
- Flywoo GOKU GN745 V3 AIO (ArduPilot firmware)
- Other ArduPilot-compatible flight controllers
"""

import time
from pymavlink import mavutil

class FlightController:
    """
    Manages MAVLink communication with ArduPilot flight controller.
    
    Optimized for Flywoo GOKU GN745 V3 AIO:
    - STM32F745 processor running ArduPilot
    - 7 UART ports available
    - Standard baud rate: 57600
    - Can power Raspberry Pi via 5V/2A BEC
    """
    
    def __init__(self, port='/dev/ttyAMA0', baud=57600):
        """
        Initialize flight controller connection.
        
        Args:
            port: Serial port path (default: /dev/ttyAMA0 for Raspberry Pi GPIO)
                  For Flywoo GOKU GN745 V3, connect to any available UART port
            baud: Baud rate (default: 57600, standard for ArduPilot)
        """
        self.port = port
        self.baud = baud
        self.connection = None
        self.connected = False
        self.vehicle_state = {
            'position': {'x': 0, 'y': 0, 'z': 0},
            'velocity': {'vx': 0, 'vy': 0, 'vz': 0},
            'heading': 0,
            'armed': False,
            'mode': None
        }
    
    def connect_to_fc(self, port=None, baud=None):
        """
        Establish MAVLink connection to flight controller.
        
        For Flywoo GOKU GN745 V3 AIO:
        - Connect Raspberry Pi UART (GPIO 14/15) to any FC UART port
        - Ensure ArduPilot firmware is configured for telemetry on that UART
        - Default baud rate 57600 is standard
        
        Args:
            port: Optional port override
            baud: Optional baud rate override
            
        Returns:
            bool: True if connection successful, False otherwise
        """
        if port:
            self.port = port
        if baud:
            self.baud = baud
        
        try:
            print(f"[NAV] Connecting to flight controller (Flywoo GOKU GN745 V3) at {self.port} ({self.baud} baud)...")
            print("[NAV] Note: Ensure ArduPilot is configured for telemetry on this UART port")
            self.connection = mavutil.mavlink_connection(self.port, baud=self.baud)
            
            # Wait for heartbeat to confirm connection
            print("[NAV] Waiting for heartbeat...")
            self.connection.wait_heartbeat(timeout=10)
            
            if (self.connection.target_system == 0 & self.connection.target_component == 0): 
                self.connected = False
                print("[NAV] No Flight controller heartbeat detected. Aborting...")
                return False
            
            else:
                print(f"[NAV] Connected! System: {self.connection.target_system}, Component: {self.connection.target_component}")
                print("[NAV] Flight controller ready for autonomous navigation")
                self.connected = True
                # Start updating vehicle state
                self.update_vehicle_state()
                return True
            
        except Exception as e:
            print(f"[NAV] Connection failed: {e}")
            print("[NAV] Check:")
            print("  - UART connection between Pi and FC")
            print("  - ArduPilot SERIALx_PROTOCOL set to 2 (MAVLink)")
            print("  - Correct baud rate (default: 57600)")
            print("  - Serial port permissions (may need to add user to dialout group)")
            self.connected = False
            return False
    
    def update_vehicle_state(self):
        """Update vehicle state from MAVLink messages."""
        if not self.connected:
            return
        
        try:
            # Process any pending messages
            msg = self.connection.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT'], 
                                            blocking=False, timeout=0.1)
            
            if msg:
                if msg.get_type() == 'LOCAL_POSITION_NED': # position relative to the EKF origin
                    self.vehicle_state['position']['x'] = msg.x # in m
                    self.vehicle_state['position']['y'] = msg.y # in m
                    self.vehicle_state['position']['z'] = msg.z # in m
                
                elif msg.get_type() == 'ATTITUDE':
                    # Convert yaw from radians to degrees
                    self.vehicle_state['heading'] = msg.yaw * 180.0 / 3.14159
                
                elif msg.get_type() == 'HEARTBEAT':
                    self.vehicle_state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode = msg.custom_mode
                    # Decode flight mode (simplified - ArduPilot specific)
                    self.vehicle_state['mode'] = mode
                    
        except Exception as e:
            print(f"[NAV] Error updating vehicle state: {e}")
    
    def get_vehicle_state(self):
        """
        Get current vehicle state.
        
        Returns:
            dict: Vehicle state including position, velocity, heading, armed status, mode
        """
        self.update_vehicle_state()
        return self.vehicle_state.copy()

    def set_EKF_origin(self):
        """
        Set the origin for the extended Kalman filter (EKF). This is the (0,0,0) point for the drone. 
        Because our drone does not use GPS, this must be set manually when the drone initializes, and cannot be changed afterwards.
        """

        if not self.connected:
            return False
        
        try:
            self.connection.mav.set_gps_global_origin_send(
                self.connection.target_system,
                0, # latitude in degrees, set to 0 because GPS is not used
                0, # longitude in degrees, set to 0 because GPS is not used
                0, # altitude in mm, set to 0 because GPS is not used
                0, # time stamp in ms, not used
            )
            print("[NAV] EKF origin set")
            return True
        except Exception as e:
            print(f"[NAV] Error setting EKF origin: {e}")
            return False
    
    def set_guided_mode(self):
        """
        Switch flight controller to GUIDED mode for autonomous navigation.
        
        Returns:
            bool: True if mode change successful, False otherwise
        """
        if not self.connected:
            print("[NAV] Not connected to flight controller")
            return False
        
        try:
            # GUIDED mode ID for ArduPilot (mode 4)
            mode_id = 4
            print(f"[NAV] Setting flight mode to GUIDED (mode {mode_id})...")
            
            self.connection.set_mode(mode_id)
            
            # Wait for mode change confirmation
            timeout = time.time() + 5
            while time.time() < timeout:
                self.update_vehicle_state()
                if self.vehicle_state['mode'] == mode_id:
                    print("[NAV] Successfully switched to GUIDED mode")
                    return True
                time.sleep(0.1)
            
            print("[NAV] Warning: Mode change may not have completed")
            return False
            
        except Exception as e:
            print(f"[NAV] Error setting GUIDED mode: {e}")
            return False
    
    def send_velocity_command(self, vx, vy, vz):
        """
        Send velocity command to flight controller (for visual servoing).
        Ardupilot will automatically stop moving the drone after 3 seconds of no input, so this command is kinda useless

        Args:
            vx: Forward velocity (m/s, positive = forward)
            vy: Right velocity (m/s, positive = right)
            vz: Down velocity (m/s, positive = down)
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with velocity components
            # Type mask: 0b010111000111 = ignore position, use velocity
            type_mask = int(0b010111000111)
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0, 0, 0,  # x, y, z (ignored)
                vx, vy, vz,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz (ignored)
                0, 0  # yaw, yaw_rate 
            )

            print(f"[NAV] Sending velocity command. vx: {vx}m/s, vy: {vy}m/s vz: {vz}m/s")
            return True
            
        except Exception as e:
            print(f"[NAV] Error sending velocity command: {e}")
            return False
    
    def send_absolute_position_command(self, x, y, z):
        """
        Send position command (waypoint) to flight controller, relative to the EKF origin
        
        Args:
            x: positive is forward, negative is backwards (meters)
            y: positive is right, negative is left (meters)
            z: positive is down, negative is up (meters)
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with position commands
            # Type mask: 0b110111111000 = use position only
            type_mask = int(0b110111111000)
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # coordinates are relative to the EKF origin 
                type_mask,
                x,  # position X (+forward/-back) in meters
                y,  # position Y (+right/-left) in meters
                z,  # altitude (-up/+down) in meters. 
                0, 0, 0,  # velocity (ignored)
                0, 0, 0,  # acceleration (ignored)
                0, 0  # yaw, yaw_rate (ignored)
            )
            print(f"[NAV] Sending absolute position command. New position is x: {x}m, y: {y}m z: {z}m (-z is up)")
            return True
            
        except Exception as e:
            print(f"[NAV] Error sending absolute position command: {e}")
            return False

    def send_relative_position_command(self, x, y, z):
        """
        Send position command (waypoint) to flight controller, relative to the drone's current position and heading
        
        Args:
            x: positive is forward, negative is backwards (meters)
            y: positive is right, negative is left (meters)
            z: positive is down, negative is uo (meters)
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with position commands
            # Type mask: 0b110111111000 = ignore velocity, use position
            type_mask = int(0b110111111000)
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # coordinates are relative to drone as it's currently oriented.  
                type_mask,
                x,  # position X (+forward/-back) in meters. relative to the body, so +x is directly in front of the drone
                y,  # position Y (+right/-left) in meters
                z,  # altitude (-up/+down) in meters.
                0, 0, 0,  # velocity (ignored)
                0, 0, 0,  # acceleration (ignored)
                0, 0  # yaw, yaw_rate (ignored)
            )
            print(f"[NAV] Sending relative position command. Position change is x: {x}m, y: {y}m z: {z}m (-z is up)")
            return True
            
        except Exception as e:
            print(f"[NAV] Error sending relative position command: {e}")
            return False

    def send_absolute_heading_command(self, heading):
        """
        Makes the flight controller turn to the given heading. Heading is relative to EKF origin, with positive values being clockwise
        
        Args:
            yaw: absolute heading (degrees), relative to the EKF origin. 0 is the direction that the drone was facing when it initialized
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with velocity components
            # Type mask: 0b100111000111  = use yaw, set velocity to 0
            type_mask = int(0b100111000111)
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # heading is relative to the EKF origin
                type_mask,
                0, 0, 0,  # x, y, z (ignored)
                0, 0, 0,  # vx, vy, vz (ignored)
                0, 0, 0,  # afx, afy, afz (ignored)
                heading / 57.29578, # heading, from degrees to radians
                0  # yaw_rate (ignored)
            )
            print(f"[NAV] Sending absolute heading command. New heading: {heading} degrees")
            return True
            
        except Exception as e:
            print(f"[NAV] Error sending absolute heading command: {e}")
            return False

    def send_relative_heading_command(self, heading):
        """
        Makes the flight controller turn to a new heading, relative to the drone's current heading. positive values are clockwise
        
        Args:
            yaw: new heading (degrees), relative to the drone's current heading. 0 is the direction that the drone is currently pointing in.
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with velocity components
            # Type mask: 0b100111000111  = use yaw, set velocity to 0
            type_mask = int(0b100111000111)
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # heading is relative to drone as it's currently oriented.
                type_mask,
                0, 0, 0,  # x, y, z (ignored)
                0, 0, 0,  # vx, vy, vz (ignored)
                0, 0, 0,  # afx, afy, afz (ignored)
                heading / 57.29578, # yaw, from degrees to radians
                0  # yaw_rate (ignored)
            )
            print(f"[NAV] Sending relative heading command. Heading change: {heading} degrees")
            return True

        except Exception as e:
            print(f"[NAV] Error sending relative heading command: {e}")
            return False

    def send_yaw_rate_command(self, yaw_rate):
        """
        Makes the flight controller yaw continuously. 
        Ardupilot will automatically stop moving the drone after 3 seconds of no input, so this command is kinda useless

        Args:
            yaw_rate: rotation rate (degrees/sec). positive is counter clockwise
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with velocity components
            # Type mask: 0b010111000111  = use yaw rate, set velocity to 0
            type_mask = int(0b010111000111)
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # heading is relative to the EKF origin
                type_mask,
                0, 0, 0,  # x, y, z (ignored)
                0, 0, 0,  # vx, vy, vz (ignored)
                0, 0, 0,  # afx, afy, afz (ignored)
                0,        # yaw, (ignored)
                yaw_rate / 57.29578  # yaw_rate, from degrees/sec to radians/sec 
            )
            print(f"[NAV] Sending yaw rate command. Yaw rate: {yaw_rate} degrees/sec")
            return True
            
        except Exception as e:
            print(f"[NAV] Error yaw rate command: {e}")
            return False

    def arm(self):
        """Arm the vehicle."""
        if not self.connected:
            return False
        
        try:
            self.connection.arducopter_arm()
            print("[NAV] Arming command sent")
            return True
        except Exception as e:
            print(f"[NAV] Error arming: {e}")
            return False
    
    def disarm(self):
        """Disarm the vehicle."""
        if not self.connected:
            return False
        
        try:
            self.connection.arducopter_disarm()
            print("[NAV] Disarm command sent")
            return True
        except Exception as e:
            print(f"[NAV] Error disarming: {e}")
            return False
    
    def land(self):
        """Command vehicle to land at its current position."""
        if not self.connected:
            return False
        
        try:
            # LAND mode ID for ArduPilot (mode 9)
            mode_id = 9
            self.connection.set_mode(mode_id)
            print("[NAV] Land command sent")
            return True
        except Exception as e:
            print(f"[NAV] Error sending land command: {e}")
            return False
    
    def return_to_launch(self):
        """Command vehicle to return to launch/home position."""
        if not self.connected:
            return False
        
        try:
            # RTL mode ID for ArduPilot (mode 6)
            mode_id = 6
            self.connection.set_mode(mode_id)
            print("[NAV] Return to launch command sent")
            return True
        except Exception as e:
            print(f"[NAV] Error sending RTL command: {e}")
            return False
    
    def takeoff(self, altitude=1.5):
        """
        Command vehicle to take off to specified altitude.
        
        Args:
            altitude: Target altitude in meters (default: 1.5m for indoor flight)
        
        Returns:
            bool: True if takeoff command sent successfully
        """
        if not self.connected:
            print("[NAV] Cannot takeoff: Flight controller not connected")
            return False
        
        try:
            # Update vehicle state
            self.update_vehicle_state()

            # Ensure GUIDED mode
            if not self.vehicle_state["mode"] == 4:
                print("[NAV] Vehicle not in guided mode for takeoff, sending command now...")
                self.set_guided_mode()
            
            # Ensure the vehicle is armed
            if not self.vehicle_state["armed"] == True:
                print("[NAV] Vehicle not armed for takeoff, arming now...")
                self.connection.arducopter_arm()
            
            # Wait for arming confirmation
            timeout = time.time() + 10
            while time.time() < timeout:
                self.update_vehicle_state()
                if self.vehicle_state['armed']:
                    print("[NAV] Vehicle armed successfully")
                    break
                time.sleep(0.1)
            
            if not self.vehicle_state['armed']:
                print("[NAV] Warning: Arming timeout, vehicle may not be armed")
            
            # Send takeoff command
            print(f"[NAV] Sending takeoff command to {altitude}m...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # confirmation
                0,  # param1 (minimum pitch, not used for multicopter)
                0,  # param2 (unused)
                0,  # param3 (unused)
                0,  # param4 (yaw angle, 0 = current heading)
                0,  # param5 (latitude, not used)
                0,  # param6 (longitude, not used)
                altitude  # param7 (altitude in meters)
            )
            
            print(f"[NAV] Takeoff command sent to {altitude}m")
            return True
            
        except Exception as e:
            print(f"[NAV] Error during takeoff: {e}")
            return False
    
    def emergency_disarm(self):
        """
        Immediately disarm the vehicle (motor kill switch).
        This cuts all power to motors for emergency stop.
        
        Returns:
            bool: True if disarm command sent successfully
        """
        if not self.connected:
            print("[NAV] Cannot disarm: Flight controller not connected")
            return False
        
        try:
            print("[NAV] EMERGENCY DISARM - Cutting all motor power!")
            # Force disarm immediately
            self.connection.arducopter_disarm()
            # Also send disarm command with force flag
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                0,  # param1: 0 = disarm
                21196,  # param2: force disarm (21196 = MAV_COMPONENT_ARM_DISARM_FORCE)
                0, 0, 0, 0, 0, 0  # unused params
            )
            print("[NAV] Emergency disarm command sent - motors should be off")
            return True
        except Exception as e:
            print(f"[NAV] Error during emergency disarm: {e}")
            return False
    
    def close(self):
        """Close connection to flight controller."""
        if self.connected:
            print("[NAV] Closing connection to flight controller")
            self.connected = False
            # Note: mavutil connections don't have explicit close, but we mark as disconnected

