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
            'position': {'lat': 0, 'lon': 0, 'alt': 0},
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
            
            self.connected = True
            print(f"[NAV] Connected! System: {self.connection.target_system}, Component: {self.connection.target_component}")
            print("[NAV] Flight controller ready for autonomous navigation")
            
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
            msg = self.connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'], 
                                            blocking=False, timeout=0.1)
            
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.vehicle_state['position']['lat'] = msg.lat / 1e7
                    self.vehicle_state['position']['lon'] = msg.lon / 1e7
                    self.vehicle_state['position']['alt'] = msg.relative_alt / 1000.0  # mm to meters
                
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
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate=0):
        """
        Send velocity command to flight controller (for visual servoing).
        
        Args:
            vx: Forward velocity (m/s, positive = forward)
            vy: Right velocity (m/s, positive = right)
            vz: Down velocity (m/s, positive = down)
            yaw_rate: Yaw rate (rad/s, positive = clockwise)
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_LOCAL_NED with velocity components
            # Type mask: 0b110111000111 = ignore position, use velocity
            type_mask = 0b110111000111
            
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0, 0, 0,  # x, y, z (ignored)
                vx, vy, vz,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz (ignored)
                yaw_rate, 0  # yaw, yaw_rate
            )
            return True
            
        except Exception as e:
            print(f"[NAV] Error sending velocity command: {e}")
            return False
    
    def send_position_command(self, lat, lon, alt):
        """
        Send position command (waypoint) to flight controller.
        
        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude (meters, relative to home)
        """
        if not self.connected:
            return False
        
        try:
            # Use SET_POSITION_TARGET_GLOBAL_INT
            # Type mask: 0b110111111000 = ignore velocity, use position
            type_mask = 0b110111111000
            
            self.connection.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7),  # latitude in degrees * 1e7
                int(lon * 1e7),  # longitude in degrees * 1e7
                alt,  # altitude in meters
                0, 0, 0,  # velocity (ignored)
                0, 0, 0,  # acceleration (ignored)
                0, 0  # yaw, yaw_rate (ignored)
            )
            return True
            
        except Exception as e:
            print(f"[NAV] Error sending position command: {e}")
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
        """Command vehicle to land."""
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
    
    def close(self):
        """Close connection to flight controller."""
        if self.connected:
            print("[NAV] Closing connection to flight controller")
            self.connected = False
            # Note: mavutil connections don't have explicit close, but we mark as disconnected

