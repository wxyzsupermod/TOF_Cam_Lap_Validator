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

        # Register all options
        self.rhapi.fields.register_option(i2c_bus_field, 'tof_control')
        self.rhapi.fields.register_option(multiplexer_addr_field, 'tof_control')
        self.rhapi.fields.register_option(channels_field, 'tof_control')
        self.rhapi.fields.register_option(distance_field, 'tof_control')

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
        """Select a channel on the TCA9548A multiplexer"""
        if channel > 7:
            return False
        try:
            self.bus.write_byte(TCA9548A_ADDRESS, 1 << channel)
            return True
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to select channel {channel}: {str(e)}')
            return False
    
    def scan_multiplexer(self, args=None):
        """Scan the multiplexer for connected ToF sensors"""
        try:
            # Get I2C bus number
            bus_num = int(self.rhapi.db.option('tof_i2c_bus'))
            multiplexer_addr = int(self.rhapi.db.option('tof_multiplexer_addr'), 16)
            
            self.rhapi.ui.message_notify(f'Scanning multiplexer at address 0x{multiplexer_addr:02X} on bus {bus_num}')
            
            # Initialize I2C bus
            bus = smbus2.SMBus(bus_num)
            
            # Test each channel
            working_channels = []
            for channel in range(8):
                try:
                    # Select channel
                    bus.write_byte(multiplexer_addr, 1 << channel)
                    time.sleep(0.1)  # Give the multiplexer time to switch
                    
                    # Try to detect a VL53L1X sensor
                    try:
                        # Check if something responds at the VL53L1X address (0x29)
                        bus.read_byte(0x29)
                        self.rhapi.ui.message_notify(f'Channel {channel}: VL53L1X sensor detected')
                        working_channels.append(channel)
                    except Exception:
                        self.rhapi.ui.message_notify(f'Channel {channel}: No VL53L1X sensor detected')
                except Exception as e:
                    self.rhapi.ui.message_notify(f'Channel {channel}: Error: {str(e)}')
            
            bus.close()
            
            # Update the channels option with the working channels
            if working_channels:
                channels_str = ','.join(str(c) for c in working_channels)
                self.rhapi.db.option_set('tof_channels', channels_str)
                self.rhapi.ui.message_notify(f'Updated active channels to: {channels_str}')
                return {'success': True, 'channels': working_channels}
            else:
                self.rhapi.ui.message_alert('No ToF sensors found on any channel')
                return {'success': False}
                
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to scan multiplexer: {str(e)}')
            return {'success': False, 'error': str(e)}
    
    def initialize_sensors(self):
        """Initialize all ToF sensors on the active channels"""
        try:
            # Get I2C bus number and channels
            bus_num = int(self.rhapi.db.option('tof_i2c_bus'))
            channels_str = self.rhapi.db.option('tof_channels')
            channels = [int(c.strip()) for c in channels_str.split(',') if c.strip()]
            
            # Initialize I2C bus
            self.bus = smbus2.SMBus(bus_num)
            self.sensors = []
            
            for channel in channels:
                if self.select_channel(channel):
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
                        
                        # Store the sensor with its channel
                        self.sensors.append({
                            'channel': channel,
                            'sensor': sensor
                        })
                        self.rhapi.ui.message_notify(f'Initialized ToF sensor on channel {channel}')
                    except Exception as e:
                        self.rhapi.ui.message_alert(f'Failed to initialize sensor on channel {channel}: {str(e)}')
            
            return len(self.sensors) > 0
        
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
        """Main ToF sensor scanning loop."""
        try:
            while self.is_running:
                # Initialize scan data
                scan_data = []
                detection = False
                
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
                            
                            # Add to scan data
                            scan_data.append({
                                'channel': channel,
                                'distance': distance
                            })
                            
                            # Check for detections
                            if distance < self.detection_threshold:
                                detection = True
                    except Exception as e:
                        self.rhapi.ui.message_alert(f'Error reading sensor on channel {channel}: {str(e)}')
                
                # Update scan data
                with self.scan_lock:
                    self.last_scan_data = scan_data
                
                # If we detected something in the gate area
                if detection:
                    self.rhapi.ui.message_notify('ToF detected an object')
                    self.last_detection_time = self.rhapi.server.monotonic_to_epoch_millis(
                        gevent.time.monotonic()
                    )
                
                # Sleep to avoid busy-waiting
                gevent.sleep(0.01)  # 10ms sleep
                    
        except Exception as e:
            self.rhapi.ui.message_alert(f'ToF scanning error: {str(e)}')
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