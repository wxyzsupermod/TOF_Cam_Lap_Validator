import json
from datetime import datetime
import math
import numpy as np
import cv2
import time
from eventmanager import Evt
from RHUI import UIField, UIFieldType
from Database import ProgramMethod
import gevent
import threading
from gevent import monkey; monkey.patch_all()
import ArducamDepthCamera as ac

class ToFValidator:
    def __init__(self, rhapi):
        self.rhapi = rhapi
        self.camera = None
        self.detection_threshold = None
        self.last_detection_time = None
        self.detection_window = 0.5  # Time window in seconds to match detections
        self.is_running = False
        self.scanning_greenlet = None
        self.last_depth_frame = None  # Store the latest depth frame
        self.last_confidence_frame = None  # Store the latest confidence frame
        self.depth_range = 0  # Will be set when camera starts
        self.frame_lock = threading.Lock()  # Lock for thread-safe data access
        
        # Register configuration options
        connection_field = UIField('tof_connection', 'Connection Type', UIFieldType.TEXT, 
                   value='CSI',
                   desc='Connection type (CSI or USB)')
        
        confidence_field = UIField('tof_confidence', 'Confidence Threshold', UIFieldType.BASIC_INT,
                   value='30',
                   desc='Confidence threshold for valid measurements (0-255)')
                   
        distance_field = UIField('detection_distance', 'Detection Distance (mm)', UIFieldType.BASIC_INT,
                   value='1000',
                   desc='Distance threshold for detection in millimeters')
                   
        min_pixels_field = UIField('detection_min_pixels', 'Min Detection Pixels', UIFieldType.BASIC_INT,
                   value='10',
                   desc='Minimum number of pixels below threshold to trigger detection')

        # Register all options
        self.rhapi.fields.register_option(connection_field, 'tof_control')
        self.rhapi.fields.register_option(confidence_field, 'tof_control')
        self.rhapi.fields.register_option(distance_field, 'tof_control')
        self.rhapi.fields.register_option(min_pixels_field, 'tof_control')
        
        # Create UI panel
        self.rhapi.ui.register_panel('tof_control', 'ToF Camera Control', 'settings')
        
        # Add control buttons
        self.rhapi.ui.register_quickbutton('tof_control', 'start_tof', 
                                         'Start Camera', self.start_camera)
        self.rhapi.ui.register_quickbutton('tof_control', 'stop_tof',
                                         'Stop Camera', self.stop_camera)
        self.rhapi.ui.register_quickbutton('tof_control', 'calibrate_tof',
                                         'Calibrate', self.calibrate)
        self.rhapi.ui.register_quickbutton('tof_control', 'view_tof',
                                         'View Camera', self.open_visualization)
        
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
            """Serve the ToF visualization page."""
            try:
                self.rhapi.ui.message_notify('ToF view endpoint accessed')
                return render_template('tof_viz.html')
            except Exception as e:
                self.rhapi.ui.message_alert(f'Error loading template: {str(e)}')
                return f'Error: {str(e)}'
            
        @bp.route('/tof/data')
        def tof_data():
            """Serve ToF depth data as JSON."""
            if not self.is_running:
                return jsonify({
                    'error': 'ToF camera not running',
                    'depth': None,
                    'threshold': self.detection_threshold or 1000
                })
            
            with self.frame_lock:  # Protect data access with lock
                # Convert the depth frame to a format suitable for JSON
                if self.last_depth_frame is not None:
                    # Convert depth frame to a colormap for visualization
                    # Use a scaled version of the data to reduce payload size
                    depth_frame = np.copy(self.last_depth_frame)
                    
                    # Apply confidence mask if available
                    if self.last_confidence_frame is not None:
                        depth_frame[self.last_confidence_frame < self.confidence_threshold] = 0
                    
                    # Resize to make JSON payload smaller
                    resized = cv2.resize(depth_frame, (32, 24))
                    
                    # Convert to list for JSON serialization
                    depth_data = resized.tolist()
                else:
                    depth_data = None
                    
                return jsonify({
                    'depth': depth_data,
                    'threshold': self.detection_threshold,
                    'range': self.depth_range if hasattr(self, 'depth_range') else 65535
                })
                
        # Register the blueprint
        self.rhapi.ui.blueprint_add(bp)
            
    def start_camera(self, args=None):
        """Start the ToF camera scanning process."""
        if self.is_running:
            return
            
        try:
            self.camera = ac.ArducamCamera()
            
            # Get connection type from options
            connection_type = self.rhapi.db.option('tof_connection').upper()
            
            # Open the camera with appropriate connection type
            if connection_type == 'CSI':
                ret = self.camera.open(ac.Connection.CSI, 0)
            elif connection_type == 'USB':
                ret = self.camera.open(ac.Connection.USB, 0)
            else:
                self.rhapi.ui.message_alert(f'Invalid connection type: {connection_type}')
                return
                
            if ret != 0:
                self.rhapi.ui.message_alert(f'Failed to open camera. Error code: {ret}')
                return
                
            # Start camera in depth mode
            ret = self.camera.start(ac.FrameType.DEPTH)
            if ret != 0:
                self.rhapi.ui.message_alert(f'Failed to start camera. Error code: {ret}')
                self.camera.close()
                return
                
            # Get camera range information
            self.depth_range = self.camera.getControl(ac.Control.RANGE)
            
            # Get camera info
            info = self.camera.getCameraInfo()
            self.rhapi.ui.message_notify(f'Camera initialized: {info.width}x{info.height}')
            
            # Set parameters from settings
            self.detection_threshold = int(self.rhapi.db.option('detection_distance'))
            self.confidence_threshold = int(self.rhapi.db.option('tof_confidence'))
            self.min_detection_pixels = int(self.rhapi.db.option('detection_min_pixels'))
            
            self.is_running = True
            
            # Start scanning in a separate greenlet
            self.scanning_greenlet = gevent.spawn(self.scan_loop)
            
            self.rhapi.ui.message_notify('ToF camera started')
        except Exception as e:
            self.rhapi.ui.message_alert(f'Failed to start ToF camera: {str(e)}')
            if self.camera:
                try:
                    self.camera.close()
                except:
                    pass
                self.camera = None
            
    def stop_camera(self, args=None):
        """Stop the ToF camera scanning process."""
        if not self.is_running:
            return
            
        self.is_running = False
        if self.scanning_greenlet:
            self.scanning_greenlet.kill()
            self.scanning_greenlet = None
            
        if self.camera:
            try:
                self.camera.stop()
                self.camera.close()
            except Exception as e:
                self.rhapi.ui.message_alert(f'Error stopping camera: {str(e)}')
            self.camera = None
            
        self.rhapi.ui.message_notify('ToF camera stopped')
        
    def scan_loop(self):
        """Main ToF camera scanning loop."""
        try:
            while self.is_running:
                # Request a frame from the camera (with 1000ms timeout)
                frame = self.camera.requestFrame(1000)
                
                if frame is not None and isinstance(frame, ac.DepthData):
                    depth_data = frame.depth_data
                    confidence_data = frame.confidence_data
                    
                    # Process the data for object detection
                    self.process_depth_frame(depth_data, confidence_data)
                    
                    # Store latest frames (make a copy to avoid reference issues)
                    with self.frame_lock:
                        self.last_depth_frame = depth_data.copy()
                        self.last_confidence_frame = confidence_data.copy()
                    
                    # Release the frame
                    self.camera.releaseFrame(frame)
                
                # Allow other operations to proceed
                gevent.idle()
                
        except Exception as e:
            self.rhapi.ui.message_alert(f'ToF camera error: {str(e)}')
            self.stop_camera()
    
    def process_depth_frame(self, depth_frame, confidence_frame):
        """Process the depth frame to detect objects."""
        try:
            # Apply confidence mask - ignore low confidence pixels
            valid_mask = confidence_frame >= self.confidence_threshold
            
            # Apply detection threshold to find close objects
            detection_mask = np.logical_and(
                depth_frame < self.detection_threshold,
                valid_mask
            )
            
            # Count pixels that indicate an object is present
            pixels_detected = np.sum(detection_mask)
            
            # Check if enough pixels were detected
            if pixels_detected >= self.min_detection_pixels:
                # We detected something substantial within our threshold
                self.last_detection_time = self.rhapi.server.monotonic_to_epoch_millis(
                    gevent.time.monotonic()
                )
                self.rhapi.ui.message_notify(f'ToF camera detected an object ({pixels_detected} pixels)')
                
        except Exception as e:
            self.rhapi.ui.message_alert(f'Error processing depth frame: {str(e)}')

    def open_visualization(self, args=None):
        """Open the ToF visualization."""
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

            # Mark the lap as invalid and deleted
            if args.get('lap'):
                args.get('lap').invalid = True
                args.get('lap').deleted = True
    
    def on_race_stop(self, args):
        """Handler for race stop events."""
        # Clear the last detection time when race stops
        self.stop_camera()
    
    def on_race_start(self, args):
        """Handler for race start events."""
        self.rhapi.ui.message_notify("Starting ToF camera")
        self.stop_camera()
        self.start_camera()

    def calibrate(self, args=None):
        """Run a calibration sequence by analyzing depth frames over 5 seconds."""
        if not self.is_running:
            self.start_camera()
            
        self.rhapi.ui.message_notify('Move drone through gate for calibration (5 seconds)...')
        
        # Collect measurements for 5 seconds
        min_distances = []
        start_time = gevent.time.monotonic()
        calibration_duration = 5  # seconds
        
        try:
            # Temporarily stop the scanning greenlet to avoid conflicts
            if self.scanning_greenlet:
                self.scanning_greenlet.kill()
                self.scanning_greenlet = None
            
            while gevent.time.monotonic() - start_time < calibration_duration:
                # Request a frame from the camera
                frame = self.camera.requestFrame(1000)
                
                if frame is not None and isinstance(frame, ac.DepthData):
                    depth_data = frame.depth_data
                    confidence_data = frame.confidence_data
                    
                    # Apply confidence mask
                    masked_depth = np.copy(depth_data)
                    masked_depth[confidence_data < self.confidence_threshold] = np.inf
                    
                    # Find the minimum distance (excluding invalid pixels)
                    valid_pixels = np.isfinite(masked_depth)
                    if np.any(valid_pixels):
                        min_distance = np.min(masked_depth[valid_pixels])
                        min_distances.append(min_distance)
                    
                    # Release the frame
                    self.camera.releaseFrame(frame)
                
                # Brief pause
                gevent.sleep(0.1)
                    
            if min_distances:
                # Calculate average of minimum distances
                avg_min_distance = sum(min_distances) / len(min_distances)
                
                # Set threshold to average minimum distance + 20% buffer
                self.detection_threshold = int(avg_min_distance * 1.2)
                
                # Save to options
                self.rhapi.db.option_set('detection_distance', str(self.detection_threshold))
                
                self.rhapi.ui.message_notify(
                    f'Calibration complete. Average distance: {int(avg_min_distance)}mm, ' + 
                    f'Detection threshold: {self.detection_threshold}mm'
                )
            else:
                self.rhapi.ui.message_alert('Calibration failed - no valid measurements received')
                
        except Exception as e:
            self.rhapi.ui.message_alert(f'Calibration error: {str(e)}')
        finally:
            # Restart the scanning loop
            if self.is_running and not self.scanning_greenlet:
                self.scanning_greenlet = gevent.spawn(self.scan_loop)

def initialize(rhapi):
    """Initialize the plugin."""
    # Import required modules
    try:
        import ArducamDepthCamera as ac
        print(f"ArducamDepthCamera SDK version: {ac.__version__}")
    except ImportError:
        print("ArducamDepthCamera module not found. Please install it:")
        print("pip install ArducamDepthCamera")
    
    return ToFValidator(rhapi)