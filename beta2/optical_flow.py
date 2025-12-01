"""
Optical Flow sensor interface module.
Supports common optical flow sensors like PMW3901, PX4Flow, or similar SPI/I2C-based sensors.
"""

import time
import math

class OpticalFlowSensor:
    """Interface for Optical Flow distance/velocity sensor."""
    
    def __init__(self, sensor_type='PMW3901', interface='SPI', bus=0, device=0):
        """
        Initialize Optical Flow sensor.
        
        Args:
            sensor_type: Type of sensor ('PMW3901', 'PX4Flow', or 'simulated')
            interface: Communication interface ('SPI' or 'I2C')
            bus: SPI bus number or I2C bus number (default: 0)
            device: SPI device/CS number or I2C address (default: 0)
        """
        self.sensor_type = sensor_type
        self.interface = interface
        self.bus = bus
        self.device = device
        self.sensor = None
        self.connected = False
        
        # Optical flow data
        self.delta_x = 0.0  # X movement in pixels
        self.delta_y = 0.0  # Y movement in pixels
        self.quality = 0    # Quality/surface quality (0-255)
        self.last_read_time = None
        self.read_interval = 0.01  # 100Hz (10ms between reads)
        
        # Velocity estimation (m/s)
        self.velocity_x = 0.0  # Forward velocity
        self.velocity_y = 0.0  # Right velocity
        
        # Position estimation (meters, relative to start)
        self.position_x = 0.0
        self.position_y = 0.0
        
        # Sensor parameters
        self.pixel_to_meter_scale = 0.001  # Scale factor (adjust based on height and sensor)
        self.height = 1.5  # Current height in meters (for velocity calculation)
        
        if sensor_type == 'simulated':
            self.connected = True
            print("[OPTICAL_FLOW] Running in simulated mode (no actual sensor)")
        else:
            self._initialize_sensor()
    
    def _initialize_sensor(self):
        """Initialize the actual Optical Flow sensor hardware."""
        try:
            if self.sensor_type == 'PMW3901':
                try:
                    import pmw3901
                    if self.interface == 'SPI':
                        import spidev
                        spi = spidev.SpiDev()
                        spi.open(self.bus, self.device)
                        spi.max_speed_hz = 2000000  # 2MHz
                        self.sensor = pmw3901.PMW3901(spi=spi)
                    else:
                        print("[OPTICAL_FLOW] PMW3901 only supports SPI interface")
                        raise ValueError("PMW3901 requires SPI")
                    
                    self.connected = True
                    print(f"[OPTICAL_FLOW] PMW3901 sensor initialized on SPI bus {self.bus}, device {self.device}")
                except ImportError:
                    print("[OPTICAL_FLOW] Warning: pmw3901 library not found. Install with: pip install pmw3901")
                    print("[OPTICAL_FLOW] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
                except Exception as e:
                    print(f"[OPTICAL_FLOW] Error initializing PMW3901: {e}")
                    print("[OPTICAL_FLOW] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
            
            elif self.sensor_type == 'PX4Flow':
                try:
                    # PX4Flow typically uses I2C or UART
                    if self.interface == 'I2C':
                        import smbus
                        bus = smbus.SMBus(self.bus)
                        # PX4Flow I2C address is typically 0x42
                        self.sensor = {'bus': bus, 'address': self.device if self.device else 0x42}
                        self.connected = True
                        print(f"[OPTICAL_FLOW] PX4Flow sensor initialized on I2C bus {self.bus}, address 0x{self.device:02x}")
                    else:
                        print("[OPTICAL_FLOW] PX4Flow I2C interface not fully implemented")
                        print("[OPTICAL_FLOW] Falling back to simulated mode")
                        self.sensor_type = 'simulated'
                        self.connected = True
                except Exception as e:
                    print(f"[OPTICAL_FLOW] Error initializing PX4Flow: {e}")
                    print("[OPTICAL_FLOW] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
            
            else:
                print(f"[OPTICAL_FLOW] Unknown sensor type: {self.sensor_type}")
                print("[OPTICAL_FLOW] Falling back to simulated mode")
                self.sensor_type = 'simulated'
                self.connected = True
                
        except Exception as e:
            print(f"[OPTICAL_FLOW] Error during sensor initialization: {e}")
            print("[OPTICAL_FLOW] Falling back to simulated mode")
            self.sensor_type = 'simulated'
            self.connected = True
    
    def set_height(self, height):
        """
        Set current height for velocity calculation.
        Optical flow velocity depends on height above ground.
        
        Args:
            height: Height in meters
        """
        self.height = max(0.1, height)  # Minimum 10cm
    
    def read_flow(self):
        """
        Read optical flow data from sensor.
        
        Returns:
            tuple: (delta_x, delta_y, quality) in pixels, or None if read failed
        """
        if not self.connected:
            return None
        
        # Rate limiting
        now = time.time()
        if self.last_read_time and (now - self.last_read_time) < self.read_interval:
            return (self.delta_x, self.delta_y, self.quality)
        
        try:
            if self.sensor_type == 'simulated':
                # Simulated optical flow (for testing without hardware)
                import random
                # Simulate small movements
                self.delta_x = random.uniform(-5, 5)
                self.delta_y = random.uniform(-5, 5)
                self.quality = random.randint(100, 255)
                self.last_read_time = now
                return (self.delta_x, self.delta_y, self.quality)
            
            elif self.sensor_type == 'PMW3901':
                # Read motion from PMW3901
                motion = self.sensor.get_motion()
                if motion:
                    self.delta_x = motion[0]  # X delta in pixels
                    self.delta_y = motion[1]  # Y delta in pixels
                    # PMW3901 doesn't provide quality, use default
                    self.quality = 200
                    self.last_read_time = now
                    return (self.delta_x, self.delta_y, self.quality)
                else:
                    return None
            
            elif self.sensor_type == 'PX4Flow':
                # Read from PX4Flow (simplified - would need full protocol implementation)
                # This is a placeholder - full implementation would read I2C registers
                print("[OPTICAL_FLOW] PX4Flow full implementation requires protocol details")
                return None
            
        except Exception as e:
            print(f"[OPTICAL_FLOW] Error reading flow: {e}")
            return None
        
        return None
    
    def get_velocity(self):
        """
        Get velocity estimate from optical flow.
        Velocity depends on height and pixel-to-meter conversion.
        
        Returns:
            tuple: (vx, vy) velocity in m/s, or (0, 0) if unavailable
        """
        flow_data = self.read_flow()
        if flow_data is None or self.quality < 50:  # Low quality threshold
            return (0.0, 0.0)
        
        delta_x, delta_y, quality = flow_data
        
        # Convert pixel movement to meters
        # Scale depends on height: higher = larger movement per pixel
        # Typical formula: meters = pixels * height * scale_factor
        scale = self.pixel_to_meter_scale * self.height
        
        # Convert to velocity (pixels per read_interval to m/s)
        self.velocity_x = delta_x * scale / self.read_interval
        self.velocity_y = delta_y * scale / self.read_interval
        
        return (self.velocity_x, self.velocity_y)
    
    def get_position(self):
        """
        Get position estimate (relative to start) from optical flow integration.
        
        Returns:
            tuple: (x, y) position in meters relative to start
        """
        flow_data = self.read_flow()
        if flow_data is None or self.quality < 50:
            return (self.position_x, self.position_y)
        
        delta_x, delta_y, quality = flow_data
        
        # Integrate pixel movement to position
        scale = self.pixel_to_meter_scale * self.height
        self.position_x += delta_x * scale
        self.position_y += delta_y * scale
        
        return (self.position_x, self.position_y)
    
    def reset_position(self):
        """Reset position estimate to (0, 0)."""
        self.position_x = 0.0
        self.position_y = 0.0
        print("[OPTICAL_FLOW] Position reset to origin")
    
    def is_connected(self):
        """Check if sensor is connected and working."""
        return self.connected
    
    def get_quality(self):
        """
        Get current surface quality reading.
        
        Returns:
            int: Quality value (0-255, higher is better)
        """
        flow_data = self.read_flow()
        if flow_data:
            return flow_data[2]  # quality
        return 0
    
    def close(self):
        """Close sensor connection."""
        if self.sensor and self.sensor_type != 'simulated':
            try:
                if self.sensor_type == 'PMW3901' and hasattr(self.sensor, 'close'):
                    self.sensor.close()
            except:
                pass
        self.connected = False
        print("[OPTICAL_FLOW] Sensor connection closed")

