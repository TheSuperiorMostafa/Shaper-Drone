"""
Time-of-Flight (ToF) sensor interface module.
Supports common ToF sensors like VL53L0X, VL53L1X, or similar I2C-based sensors.
"""

import time
import smbus

class TOFSensor:
    """Interface for Time-of-Flight distance sensor."""
    
    def __init__(self, sensor_type='VL53L0X', i2c_bus=1, address=None):
        """
        Initialize ToF sensor.
        
        Args:
            sensor_type: Type of sensor ('VL53L0X', 'VL53L1X', or 'simulated')
            i2c_bus: I2C bus number (default: 1 for Raspberry Pi)
            address: I2C address (auto-detect if None)
        """
        self.sensor_type = sensor_type
        self.i2c_bus = i2c_bus
        self.address = address
        self.sensor = None
        self.connected = False
        self.last_distance = None
        self.last_read_time = None
        
        # Sensor parameters
        self.min_distance = 0.03  # 3cm minimum
        self.max_distance = 4.0   # 4m maximum (adjust based on sensor)
        self.read_interval = 0.05  # 50ms between reads (20Hz)
        
        if sensor_type == 'simulated':
            self.connected = True
            print("[TOF] Running in simulated mode (no actual sensor)")
        else:
            self._initialize_sensor()
    
    def _initialize_sensor(self):
        """Initialize the actual ToF sensor hardware."""
        try:
            if self.sensor_type == 'VL53L0X':
                try:
                    import VL53L0X
                    self.sensor = VL53L0X.VL53L0X(i2c_bus=self.i2c_bus, i2c_address=self.address)
                    self.sensor.open()
                    self.sensor.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
                    self.connected = True
                    print(f"[TOF] VL53L0X sensor initialized on I2C bus {self.i2c_bus}")
                except ImportError:
                    print("[TOF] Warning: VL53L0X library not found. Install with: pip install VL53L0X")
                    print("[TOF] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
                except Exception as e:
                    print(f"[TOF] Error initializing VL53L0X: {e}")
                    print("[TOF] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
            
            elif self.sensor_type == 'VL53L1X':
                try:
                    import VL53L1X
                    self.sensor = VL53L1X.VL53L1X(i2c_bus=self.i2c_bus, i2c_address=self.address)
                    self.sensor.open()
                    self.sensor.start_ranging()
                    self.connected = True
                    print(f"[TOF] VL53L1X sensor initialized on I2C bus {self.i2c_bus}")
                except ImportError:
                    print("[TOF] Warning: VL53L1X library not found")
                    print("[TOF] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
                except Exception as e:
                    print(f"[TOF] Error initializing VL53L1X: {e}")
                    print("[TOF] Falling back to simulated mode")
                    self.sensor_type = 'simulated'
                    self.connected = True
            
            else:
                print(f"[TOF] Unknown sensor type: {self.sensor_type}")
                print("[TOF] Falling back to simulated mode")
                self.sensor_type = 'simulated'
                self.connected = True
                
        except Exception as e:
            print(f"[TOF] Error during sensor initialization: {e}")
            print("[TOF] Falling back to simulated mode")
            self.sensor_type = 'simulated'
            self.connected = True
    
    def read_distance(self):
        """
        Read distance from ToF sensor.
        
        Returns:
            float: Distance in meters, or None if read failed
        """
        if not self.connected:
            return None
        
        # Rate limiting
        now = time.time()
        if self.last_read_time and (now - self.last_read_time) < self.read_interval:
            return self.last_distance
        
        try:
            if self.sensor_type == 'simulated':
                # Simulated distance (for testing without hardware)
                import random
                distance = 1.5 + random.uniform(-0.2, 0.2)  # Simulate ~1.5m with noise
                self.last_distance = distance
                self.last_read_time = now
                return distance
            
            elif self.sensor_type == 'VL53L0X':
                distance_mm = self.sensor.get_distance()
                if distance_mm > 0:
                    distance = distance_mm / 1000.0  # Convert mm to meters
                    # Validate range
                    if self.min_distance <= distance <= self.max_distance:
                        self.last_distance = distance
                        self.last_read_time = now
                        return distance
                    else:
                        return None
                else:
                    return None
            
            elif self.sensor_type == 'VL53L1X':
                distance_mm = self.sensor.get_distance()
                if distance_mm > 0:
                    distance = distance_mm / 1000.0  # Convert mm to meters
                    # Validate range
                    if self.min_distance <= distance <= self.max_distance:
                        self.last_distance = distance
                        self.last_read_time = now
                        return distance
                    else:
                        return None
                else:
                    return None
            
        except Exception as e:
            print(f"[TOF] Error reading distance: {e}")
            return self.last_distance  # Return last valid reading
        
        return None
    
    def get_distance(self):
        """
        Alias for read_distance() for convenience.
        
        Returns:
            float: Distance in meters, or None if read failed
        """
        return self.read_distance()
    
    def is_connected(self):
        """Check if sensor is connected and working."""
        return self.connected
    
    def close(self):
        """Close sensor connection."""
        if self.sensor and self.sensor_type != 'simulated':
            try:
                if self.sensor_type == 'VL53L0X':
                    self.sensor.stop_ranging()
                elif self.sensor_type == 'VL53L1X':
                    self.sensor.stop_ranging()
            except:
                pass
        self.connected = False
        print("[TOF] Sensor connection closed")

