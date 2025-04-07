import json
from datetime import datetime
import math
import smbus2
import time
from VL53L1X import VL53L1X
from eventmanager import Evt
from RHUI import UIField, UIFieldType
from Database import ProgramMethod
import gevent
import threading
from gevent import monkey; monkey.patch_all()

# I2C multiplexer address
TCA9548A_ADDRESS = 0x70

class ToFValidator:
    def __init__(self, rhapi):
        self.rhapi = rhapi
        self.sensors = []
        self.bus = None
        self.detection_threshold = None
        self.last_detection_time = None
        self.detection_window = 0.5  # Time window in seconds to match detections
        self.is_running = False
        self.scanning_greenlet = None
        self.last_scan_data = []  # Initialize empty list for scan data
        self.scan_lock = threading.Lock()  # Add a lock for thread-safe data access
        self.working_channels = []
        self.direct_mode = False  # If True, bypass multiplexer and connect directly to sensors
        
        # Add utility function to dump module version information
        try:
            import pkg_resources
            modules = ['smbus2', 'VL53L1X', 'RPi.GPIO']
            versions = {}
            for module in modules:
                try:
                    versions[module] = pkg_resources.get_distribution(module).version
                except:
                    versions[module] = "Not installed"
            self.rhapi.ui.message_notify(f"Module versions: {versions}")
        except:
            pass
        
        # Register options
        i2c_bus_field = UIField('tof_i2c_bus', 'I2C Bus Number', UIFieldType.BASIC_INT, 
                    value='1',
                    desc='I2C bus number (usually 1)')
        
        multiplexer_addr_field = UIField('tof_multiplexer_addr', 'Multiplexer Address', UIFieldType.TEXT, 
                    value='0x70',
                    desc='I2C address of TCA9548A multiplexer (in hex)')
        
        channels_field = UIField('tof_channels', 'Active Channels', UIFieldType.TEXT,
                    value='0,1,2,3',
                    desc='Comma-separated list of multiplexer channels to use')
        
        distance_field = UIField('detection_distance', 'Detection Distance (mm)', UIFieldType.BASIC_INT,
                    value='1000',
                    desc='Distance threshold for detection in millimeters')
                    
        direct_mode_field = UIField('tof_direct_mode', 'Direct Mode (No Multiplexer)', UIFieldType.CHECKBOX,
                    value=False,
                    desc='Enable to bypass multiplexer and connect directly to a single VL53L1X sensor')

        # Register all options
        self.rhapi.fields.register_option(i2c_bus_field, 'tof_control')
        self.rhapi.fields.register_option(multiplexer_addr_field, 'tof_control')
        self.rhapi.fields.register_option(channels_field, 'tof_control')
        self.rhapi.fields.register_option(distance_field, 'tof_control')
        self.rhapi.fields.register_option(direct_mode_field, 'tof_control')
        
        # Add help text with markdown
        self.rhapi.ui.register_markdown('tof_control', 'tof_help', """
## ToF Sensor Troubleshooting

If you're seeing I/O errors:

1. **Check I2C Bus**:
   - Run `i2cdetect -y 1` to see connected devices
   - Your multiplexer should appear (usually at 0x70)

2. **Check Connections**:
   - VCC: 3.3V power
   - GND: Ground
   - SDA: Serial Data (GPIO 2 on Pi)
   - SCL: Serial Clock (GPIO 3 on Pi)

3. **Try Direct Mode**:
   - Enable "Direct Mode" to connect directly to a single sensor
   - Connect VL53L1X directly to I2C pins (no multiplexer)

4. **Hardware Requirements**:
   - TCA9548A I2C multiplexer
   - VL53L1X ToF sensors
   - Pull-up resistors (2.2kΩ to 10kΩ) on SDA/SCL lines
""")

        # Create UI panel
        self.rhapi.ui.register_panel('tof_control', 'ToF Sensor Control', 'settings')
        
        # Add control buttons
        self.rhapi.ui.register_quickbutton('tof_control', 'start_sensors', 
                                         'Start Sensors', self.start_sensors)
        self.rhapi.ui.register_quickbutton('tof_control', 'stop_sensors',
                                         'Stop Sensors', self.stop_sensors)
        self.rhapi.ui.register_quickbutton('tof_control', 'calibrate_sensors',
                                         'Calibrate', self.calibrate)
        self.rhapi.ui.register_quickbutton('tof_control', 'view_sensors',
                                         'View Sensors', self.open_visualization)
        self.rhapi.ui.register_quickbutton('tof_control', 'scan_multiplexer',
                                         'Scan Multiplexer', self.scan_multiplexer)
        
        # Register event handlers
        self.rhapi.events.on(Evt.RACE_LAP_RECORDED, self.on_lap_recorded)
        self.rhapi.events.on(Evt.RACE_STOP, self.on_race_stop)
        self.rhapi.events.on(Evt.RACE_START, self.on_race_start)
        
        # Register the visualization page and API endpoint
        from flask import Blueprint, jsonify, render_template
        import os

        # Get the directory where this plugin file is located
        plugin_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Create blueprint with absolute paths
        bp = Blueprint(
            'tof_viz',
            __name__,
            template_folder=os.path.join(plugin_dir, 'templates'),
            static_folder=os.path.join(plugin_dir, 'static'),
            static_url_path='/static/tof-viz' 
        )
        
        @bp.route('/tof')
        def tof_view():
            """Serve the TOF visualization page."""
            try:
                self.rhapi.ui.message_notify('ToF view endpoint accessed')
                return render_template('tof_viz.html')
            except Exception as e:
                self.rhapi.ui.message_alert(f'Error loading template: {str(e)}')
                return f'Error: {str(e)}'
            
        @bp.route('/tof/data')
        def tof_data():
            """Serve ToF sensor data as JSON."""
            if not self.is_running:
                return jsonify({
                    'error': 'Sensors not running',
                    'scan': [],
                    'threshold': self.detection_threshold or 1000
                })
            
            with self.scan_lock:  # Protect data access with lock
                return jsonify({
                    'scan': self.last_scan_data,
                    'threshold': self.detection_threshold
                })
                
        # Register the blueprint
        self.rhapi.ui.blueprint_add(bp)
            
    def select_channel(self, channel):
        """Select a channel on the TCA9548A multiplexer with robust error handling"""
        if channel > 7:
            return False
            
        # Try multiple times to select the channel
        for retry in range(3):
            try:
                # Calculate the channel value
                channel_value = 1 << channel
                
                # Try to write to the multiplexer
                self.bus.write_byte(TCA9548A_ADDRESS, channel_value)
                
                # Give it time to switch
                time.sleep(0.05)
                
                # To validate the selection worked, try reading back the current value
                # (This may not work on all multiplexers, so we catch exceptions)
                try:
                    current_value = self.bus.read_byte(TCA9548A_ADDRESS)
                    if current_value == channel_value:
                        return True
                except:
                    # If we can't read back, just assume it worked
                    return True
                    
                return True
            except Exception as e:
                if retry < 2:  # Don't log on first retries
                    time.sleep(0.1 * (retry + 1))  # Increasing delay for retries
                else:
                    self.rhapi.ui.message_alert(f'Failed to select channel {channel} (retry {retry+1}): {str(e)}')
        
        return False
    
    def scan_multiplexer(self, args=None):
        """Scan the multiplexer for connected ToF sensors with enhanced diagnostics"""
        bus = None
        try:
            # Get I2C bus number
            bus_num = int(self.rhapi.db.option('tof_i2c_bus'))
            multiplexer_addr = int(self.rhapi.db.option('tof_multiplexer_addr'), 16)
            
            self.rhapi.ui.message_notify(f'Scanning multiplexer at address 0x{multiplexer_addr:02X} on bus {bus_num}')
            
            # Check if I2C tools are installed
            self.rhapi.ui.message_notify('Checking for I2C devices on bus...')
            try:
                import subprocess
                result = subprocess.run(['i2cdetect', '-y', str(bus_num)], 
                                       capture_output=True, text=True, timeout=5)
                self.rhapi.ui.message_notify(f'I2C devices found: \n{result.stdout}')
            except Exception as e:
                self.rhapi.ui.message_notify(f'Could not run i2cdetect: {str(e)}')
            
            # Initialize I2C bus with timeout and retries
            self.rhapi.ui.message_notify('Initializing I2C bus...')
            bus = smbus2.SMBus(bus_num)
            
            # First test if we can communicate with the multiplexer at all
            self.rhapi.ui.message_notify('Testing multiplexer communication...')
            try:
                # Try to read from the multiplexer
                value = bus.read_byte(multiplexer_addr)
                self.rhapi.ui.message_notify(f'Multiplexer responded with value: 0x{value:02X}')
            except Exception as e:
                self.rhapi.ui.message_alert(f'Failed to communicate with multiplexer: {str(e)}')
                if 'No such device' in str(e):
                    self.rhapi.ui.message_alert(f'Multiplexer not found at address 0x{multiplexer_addr:02X}. Check connections and address.')
                    return {'success': False, 'error': 'Multiplexer not found'}
                
            # Try to reset the multiplexer by writing 0
            try:
                bus.write_byte(multiplexer_addr, 0)
                time.sleep(0.1)  # Give it time to reset
                self.rhapi.ui.message_notify('Successfully reset multiplexer')
            except Exception as e:
                self.rhapi.ui.message_alert(f'Failed to reset multiplexer: {str(e)}')
            
            # Test each channel with extended timeout
            working_channels = []
            for channel in range(8):
                self.rhapi.ui.message_notify(f'Testing channel {channel}...')
                try:
                    # Select channel with retry logic
                    success = False
                    for retry in range(3):  # Try 3 times
                        try:
                            channel_value = 1 << channel
                            bus.write_byte(multiplexer_addr, channel_value)
                            self.rhapi.ui.message_notify(f'  Selected channel {channel} (value: 0x{channel_value:02X})')
                            success = True
                            break
                        except Exception as e:
                            self.rhapi.ui.message_notify(f'  Retry {retry+1} failed: {str(e)}')
                            time.sleep(0.2)  # Wait before retry
                    
                    if not success:
                        self.rhapi.ui.message_notify(f'  Failed to select channel {channel} after 3 retries')
                        continue
                        
                    # Give the multiplexer extra time to switch
                    time.sleep(0.3)
                    
                    # Try to detect VL53L1X sensor (address 0x29)
                    try:
                        # Check if something responds at the address
                        bus.write_quick(0x29)  # Less intrusive than read_byte
                        self.rhapi.ui.message_notify(f'  Channel {channel}: VL53L1X sensor detected!')
                        working_channels.append(channel)
                    except Exception as e:
                        self.rhapi.ui.message_notify(f'  Channel {channel}: No VL53L1X sensor detected ({str(e)})')
                        
                except Exception as e:
                    self.rhapi.ui.message_notify(f'  Channel {channel} test failed: {str(e)}')
            
            # Update the channels option with the working channels
            if working_channels:
                channels_str = ','.join(str(c) for c in working_channels)
                self.rhapi.db.option_set('tof_channels', channels_str)
                self.rhapi.ui.message_notify(f'Updated active channels to: {channels_str}')
                return {'success': True, 'channels': working_channels}
            else:
                self.rhapi.ui.message_alert('No ToF sensors found on any channel')
                self.rhapi.ui.message_notify('Try checking:')
                self.rhapi.ui.message_notify('1. Power connections to sensors')
                self.rhapi.ui.message_notify('2. Data (SDA/SCL) connections')
                self.rhapi.ui.message_notify('3. Multiplexer address (usually 0x70)')
                self.rhapi.ui.message_notify('4. I2C bus number (usually 1)')
                return {'success': False}
                
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to scan multiplexer: {str(e)}')
            return {'success': False, 'error': str(e)}
        finally:
            # Ensure we close the bus
            if bus:
                try:
                    bus.close()
                    self.rhapi.ui.message_notify('I2C bus closed')
                except:
                    pass
    
    def initialize_sensors(self):
        """Initialize all ToF sensors on the active channels with direct mode support"""
        try:
            # Get I2C bus number
            bus_num = int(self.rhapi.db.option('tof_i2c_bus'))
            
            # Check if direct mode is enabled
            self.direct_mode = self.rhapi.db.option('tof_direct_mode').lower() == 'true'
            
            # Initialize I2C bus
            self.bus = smbus2.SMBus(bus_num)
            self.sensors = []
            
            if self.direct_mode:
                # In direct mode, we just connect to a single sensor directly
                self.rhapi.ui.message_notify('Direct mode enabled - connecting directly to VL53L1X sensor')
                try:
                    # Initialize sensor
                    sensor = VL53L1X(i2c_bus=self.bus)
                    sensor.open()
                    # Set timing budget (33ms is default)
                    sensor.set_timing_budget_in_ms(33)
                    # Set inter-measurement period (must be >= timing budget)
                    sensor.set_inter_measurement_in_ms(33)
                    # Start ranging
                    sensor.start_ranging()
                    
                    # Store the sensor
                    self.sensors.append({
                        'channel': -1,  # -1 indicates direct mode
                        'sensor': sensor
                    })
                    self.rhapi.ui.message_notify('Initialized single ToF sensor in direct mode')
                except Exception as e:
                    self.rhapi.ui.message_alert(f'Failed to initialize sensor in direct mode: {str(e)}')
                    
            else:
                # Normal multiplexer mode
                channels_str = self.rhapi.db.option('tof_channels')
                channels = [int(c.strip()) for c in channels_str.split(',') if c.strip()]
                
                self.rhapi.ui.message_notify(f'Initializing {len(channels)} sensors via multiplexer')
                
                # First, reset the multiplexer
                try:
                    self.bus.write_byte(TCA9548A_ADDRESS, 0)
                    time.sleep(0.1)
                except Exception as e:
                    self.rhapi.ui.message_alert(f'Failed to reset multiplexer: {str(e)}')
                
                # Try to initialize each sensor
                for channel in channels:
                    self.rhapi.ui.message_notify(f'Initializing sensor on channel {channel}...')
                    if self.select_channel(channel):
                        try:
                            # Initialize sensor
                            sensor = VL53L1X(i2c_bus=self.bus)
                            
                            # Open the sensor with debug info
                            self.rhapi.ui.message_notify(f'  Opening sensor on channel {channel}...')
                            sensor.open()
                            
                            # Configure sensor
                            self.rhapi.ui.message_notify(f'  Configuring sensor on channel {channel}...')
                            sensor.set_timing_budget_in_ms(33)
                            sensor.set_inter_measurement_in_ms(33)
                            
                            # Start ranging
                            self.rhapi.ui.message_notify(f'  Starting ranging on channel {channel}...')
                            sensor.start_ranging()
                            
                            # Store the sensor with its channel
                            self.sensors.append({
                                'channel': channel,
                                'sensor': sensor
                            })
                            self.rhapi.ui.message_notify(f'  Successfully initialized ToF sensor on channel {channel}')
                        except Exception as e:
                            self.rhapi.ui.message_alert(f'Failed to initialize sensor on channel {channel}: {str(e)}')
            
            # Return success if we initialized at least one sensor
            if len(self.sensors) > 0:
                self.rhapi.ui.message_notify(f'Successfully initialized {len(self.sensors)} ToF sensors')
                return True
            else:
                self.rhapi.ui.message_alert('Failed to initialize any ToF sensors')
                return False
        
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to initialize sensors: {str(e)}')
            return False
        
    def start_sensors(self, args=None):
        """Start the ToF sensor scanning process."""
        if self.is_running:
            return
            
        try:
            self.detection_threshold = int(self.rhapi.db.option('detection_distance'))
            
            if self.initialize_sensors():
                self.is_running = True
                
                # Start scanning in a separate greenlet
                self.scanning_greenlet = gevent.spawn(self.scan_loop)
                
                self.rhapi.ui.message_notify(f'ToF sensors started ({len(self.sensors)} sensors)')
            else:
                self.rhapi.ui.message_alert('No ToF sensors were initialized')
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to start ToF sensors: {str(e)}')
            
    def stop_sensors(self, args=None):
        """Stop the ToF sensor scanning process."""
        if not self.is_running:
            return
            
        self.is_running = False
        if self.scanning_greenlet:
            self.scanning_greenlet.kill()
            self.scanning_greenlet = None
            
        # Stop and close all sensors
        for sensor_info in self.sensors:
            try:
                channel = sensor_info['channel']
                sensor = sensor_info['sensor']
                if self.select_channel(channel):
                    sensor.stop_ranging()
                    sensor.close()
            except Exception as e:
                self.rhapi.ui.message_alert(f'Error stopping sensor on channel {channel}: {str(e)}')
        
        self.sensors = []
        
        # Close I2C bus
        if self.bus:
            try:
                self.bus.close()
                self.bus = None
            except Exception as e:
                self.rhapi.ui.message_alert(f'Error closing I2C bus: {str(e)}')
            
        self.rhapi.ui.message_notify('ToF sensors stopped')
        
    def scan_loop(self):
        """Main ToF sensor scanning loop with improved error handling."""
        error_count = 0
        max_errors = 10  # Max consecutive errors before stopping
        
        try:
            self.rhapi.ui.message_notify('ToF scanning loop started')
            
            while self.is_running:
                # Initialize scan data
                scan_data = []
                detection = False
                
                try:
                    # Loop through all sensors
                    for sensor_info in self.sensors:
                        channel = sensor_info['channel']
                        sensor = sensor_info['sensor']
                        
                        # In direct mode (channel = -1), we don't need to select a channel
                        if channel >= 0 and not self.direct_mode:
                            if not self.select_channel(channel):
                                continue
                        
                        try:
                            # Check if data is ready (with timeout handling)
                            data_ready = False
                            try:
                                data_ready = sensor.check_for_data_ready()
                            except Exception as e:
                                if 'timeout' in str(e).lower():
                                    # This is normal, just skip this reading
                                    continue
                                else:
                                    # Re-raise other errors
                                    raise
                                
                            if data_ready:
                                # Get the distance
                                distance = sensor.get_distance()
                                
                                # Clear the interrupt
                                sensor.clear_interrupt()
                                
                                # Add to scan data
                                scan_data.append({
                                    'channel': channel,
                                    'distance': distance
                                })
                                
                                # Check for detections
                                if distance < self.detection_threshold:
                                    detection = True
                                    
                        except Exception as e:
                            channel_desc = "direct" if channel == -1 else f"channel {channel}"
                            self.rhapi.ui.message_alert(f'Error reading sensor on {channel_desc}: {str(e)}')
                            # Don't spam the UI with repeated errors
                            gevent.sleep(1)
                    
                    # Reset error count on successful iteration
                    error_count = 0
                    
                    # Update scan data
                    with self.scan_lock:
                        self.last_scan_data = scan_data
                    
                    # If we detected something in the gate area
                    if detection:
                        self.rhapi.ui.message_notify('ToF detected an object')
                        self.last_detection_time = self.rhapi.server.monotonic_to_epoch_millis(
                            gevent.time.monotonic()
                        )
                
                except Exception as e:
                    error_count += 1
                    self.rhapi.ui.message_alert(f'Error in scan loop (error #{error_count}): {str(e)}')
                    
                    if error_count >= max_errors:
                        self.rhapi.ui.message_alert(f'Too many consecutive errors ({max_errors}), stopping sensors')
                        self.stop_sensors()
                        break
                    
                    # Longer sleep after an error
                    gevent.sleep(0.5)
                    continue
                
                # Sleep to avoid busy-waiting
                gevent.sleep(0.01)  # 10ms sleep
                    
        except Exception as e:
            self.rhapi.ui.message_alert(f'Fatal ToF scanning error: {str(e)}')
            self.stop_sensors()

    def open_visualization(self, args=None):
        """Open the ToF sensor visualization."""
        try:
            # Return JavaScript that will be executed on the client side
            return {'script': 'window.open("/tof", "_blank")'}
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to open visualization: {str(e)}')
            return False
    
    def on_lap_recorded(self, args):
        """Handler for lap recording events."""
        if not self.is_running or not self.last_detection_time:
            return

        lap_time = args.get('lap').lap_time_stamp if args.get('lap') else 0

        # Compare timestamps
        time_diff = abs(lap_time - self.last_detection_time) / 1000.0  # Convert to seconds

        if time_diff > self.detection_window:
            # Invalid lap - no ToF detection within window
            self.rhapi.ui.message_notify(
                f'Warning: Lap recorded without ToF validation (diff: {time_diff:.2f}s)'
            )

            # Get the pilot_id and lap_number from the args
            args.get('lap').invalid = True
            args.get('lap').deleted = True
    
    def on_race_stop(self, args):
        """Handler for race stop events."""
        # Clear the last detection time when race stops
        self.stop_sensors()
    
    def on_race_start(self, args):
        """Handler for race start events."""
        self.rhapi.ui.message_notify("Starting ToF sensors")
        self.stop_sensors()
        self.start_sensors()

    def calibrate(self, args=None):
        """Run a calibration sequence by averaging distances over 5 seconds."""
        if not self.is_running:
            self.start_sensors()
            
        self.rhapi.ui.message_notify('Move drone through gate for calibration...')
        
        # Collect measurements for 5 seconds
        distances = []
        start_time = gevent.time.monotonic()
        calibration_duration = 5  # seconds
        
        try:
            while gevent.time.monotonic() - start_time < calibration_duration:
                # Loop through all sensors
                for sensor_info in self.sensors:
                    channel = sensor_info['channel']
                    sensor = sensor_info['sensor']
                    
                    # Select the channel for this sensor
                    if not self.select_channel(channel):
                        continue
                    
                    try:
                        # Check if data is ready
                        if sensor.check_for_data_ready():
                            # Get the distance
                            distance = sensor.get_distance()
                            
                            # Clear the interrupt
                            sensor.clear_interrupt()
                            
                            # Add to distances
                            distances.append(distance)
                    except Exception as e:
                        self.rhapi.ui.message_alert(f'Error reading sensor on channel {channel}: {str(e)}')
                
                gevent.sleep(0.01)  # Small delay to avoid busy loop
                    
            if distances:
                # Calculate average distance
                avg_distance = sum(distances) / len(distances)
                
                # Set threshold to average distance + 20% buffer
                self.detection_threshold = int(avg_distance * 1.2)
                
                # Save to options
                self.rhapi.db.option_set('detection_distance', str(self.detection_threshold))
                
                self.rhapi.ui.message_notify(
                    f'Calibration complete. Average distance: {int(avg_distance)}mm, ' + 
                    f'Detection threshold: {self.detection_threshold}mm'
                )
            else:
                self.rhapi.ui.message_alert('Calibration failed - no measurements received')
                
        except Exception as e:
            self.rhapi.ui.message_alert(f'Calibration error: {str(e)}')

def initialize(rhapi):
    """Initialize the plugin."""
    return ToFValidator(rhapi)