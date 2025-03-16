// Destructure React hooks from React
const { useState, useEffect, useRef } = React;

// ToF Camera Visualization Component
const ToFVisualization = () => {
  const [depthData, setDepthData] = useState(null);
  const [threshold, setThreshold] = useState(1000);
  const [maxRange, setMaxRange] = useState(65535);
  const [error, setError] = useState(null);
  const [isActive, setIsActive] = useState(false);
  const [lastUpdate, setLastUpdate] = useState(Date.now());
  const [detectionCount, setDetectionCount] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch('/tof/data');
        const data = await response.json();
        
        if (data.error) {
          setError(data.error);
          setIsActive(false);
        } else {
          if (data.depth) {
            // Check if there are any close objects (below threshold)
            let detections = 0;
            for (let y = 0; y < data.depth.length; y++) {
              for (let x = 0; x < data.depth[0].length; x++) {
                if (data.depth[y][x] > 0 && data.depth[y][x] < data.threshold) {
                  detections++;
                }
              }
            }
            setDetectionCount(detections);
          }
          
          setDepthData(data.depth);
          setThreshold(data.threshold);
          setMaxRange(data.range || 65535);
          setIsActive(true);
          setLastUpdate(Date.now());
        }
      } catch (err) {
        setError('Failed to fetch ToF data: ' + err.message);
        setIsActive(false);
      }
    };

    fetchData();
    const interval = setInterval(fetchData, 200); // Update every 200ms
    return () => clearInterval(interval);
  }, []);

  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !depthData) return;

    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    // Clear canvas
    ctx.fillStyle = '#f8f9fa';
    ctx.fillRect(0, 0, width, height);
    
    // Get depth data dimensions
    const dataHeight = depthData.length;
    const dataWidth = depthData[0].length;
    
    // Calculate cell size to fit the canvas
    const cellWidth = width / dataWidth;
    const cellHeight = height / dataHeight;
    
    // Find min and max values for color scaling
    let minVal = Infinity;
    let maxVal = 0;
    
    for (let y = 0; y < dataHeight; y++) {
      for (let x = 0; x < dataWidth; x++) {
        const value = depthData[y][x];
        if (value > 0) {  // Ignore zero values (often invalid readings)
          minVal = Math.min(minVal, value);
          maxVal = Math.max(maxVal, value);
        }
      }
    }
    
    // Ensure reasonable min/max values
    if (minVal === Infinity) minVal = 0;
    if (maxVal === 0) maxVal = maxRange;
    
    // Create a rainbow colormap (similar to cv2.COLORMAP_RAINBOW)
    const getColorFromDepth = (value) => {
      if (value <= 0) return 'rgba(0, 0, 0, 0.2)'; // Semi-transparent black for invalid readings
      
      // If below threshold, use red intensity to indicate detection strength
      if (value < threshold) {
        const intensity = Math.max(0, Math.min(255, Math.floor(255 * (value / threshold))));
        return `rgba(255, ${intensity}, ${intensity}, 0.8)`;
      }
      
      // Apply rainbow colormap for valid values
      const normalizedValue = Math.min(1, Math.max(0, 1 - (value / maxVal)));
      const hue = normalizedValue * 270; // 270 degrees of hue for rainbow effect
      return `hsla(${hue}, 100%, 50%, 0.8)`;
    };
    
    // Draw depth map using rainbow colormap
    for (let y = 0; y < dataHeight; y++) {
      for (let x = 0; x < dataWidth; x++) {
        const value = depthData[y][x];
        
        // Determine color for this pixel
        ctx.fillStyle = getColorFromDepth(value);
        ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
        
        // Draw grid lines (optional)
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.1)';
        ctx.strokeRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
      }
    }
    
    // Add depth scale reference
    const legendHeight = 20;
    const legendWidth = width - 40;
    const legendX = 20;
    const legendY = height - legendHeight - 20;
    
    // Create gradient for the scale
    const gradient = ctx.createLinearGradient(legendX, 0, legendX + legendWidth, 0);
    
    // Add rainbow colors
    gradient.addColorStop(0, 'hsl(270, 100%, 50%)'); // Far
    gradient.addColorStop(0.25, 'hsl(180, 100%, 50%)');
    gradient.addColorStop(0.5, 'hsl(135, 100%, 50%)');
    gradient.addColorStop(0.75, 'hsl(90, 100%, 50%)');
    gradient.addColorStop(1, 'hsl(0, 100%, 50%)');   // Near
    
    // Draw the gradient scale
    ctx.fillStyle = gradient;
    ctx.fillRect(legendX, legendY, legendWidth, legendHeight);
    
    // Add scale markers
    ctx.fillStyle = '#000';
    ctx.font = '10px Arial';
    ctx.textAlign = 'center';
    
    // Calculate scale intervals
    const numIntervals = 5;
    for (let i = 0; i <= numIntervals; i++) {
      const x = legendX + (i / numIntervals) * legendWidth;
      const value = Math.round(maxVal - (i / numIntervals) * maxVal);
      
      // Draw scale marker
      ctx.fillText(value + 'mm', x, legendY + legendHeight + 15);
      
      // Draw tick mark
      ctx.beginPath();
      ctx.moveTo(x, legendY + legendHeight);
      ctx.lineTo(x, legendY + legendHeight + 5);
      ctx.stroke();
    }
    
    // Draw threshold line
    const thresholdX = legendX + legendWidth * (1 - threshold / maxVal);
    ctx.beginPath();
    ctx.moveTo(thresholdX, legendY - 5);
    ctx.lineTo(thresholdX, legendY + legendHeight + 5);
    ctx.strokeStyle = '#FF0000';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.lineWidth = 1;
    
    // Add threshold label
    ctx.fillStyle = '#FF0000';
    ctx.textAlign = 'center';
    ctx.fillText('Threshold', thresholdX, legendY - 10);
    
  }, [depthData, threshold, maxRange]);

  // Calculate time since last update
  const timeSinceUpdate = Math.floor((Date.now() - lastUpdate) / 1000);
  const isStale = timeSinceUpdate > 2;

  return (
    <div className="tof-container">
      <h2>
        ToF Camera Visualization
        <span 
          className={`status-indicator ${isActive && !isStale ? 'status-active' : 'status-inactive'}`} 
          title={isActive ? 'Camera active' : 'Camera inactive'}
        ></span>
      </h2>
      
      {error ? (
        <div style={{ color: 'red' }}>{error}</div>
      ) : (
        <div>
          {isStale && isActive && (
            <div style={{ color: 'orange', marginBottom: '10px' }}>
              Warning: Data may be stale. Last update: {timeSinceUpdate} seconds ago.
            </div>
          )}
          
          <div className="controls">
            <div><strong>Threshold Distance:</strong> {threshold}mm</div>
            <div><strong>Detection Status:</strong> {
              detectionCount > 0 
                ? <span style={{color: 'red'}}>Object Detected ({detectionCount} pixels)</span> 
                : <span style={{color: 'green'}}>No Detection</span>
            }</div>
          </div>
          
          <canvas 
            ref={canvasRef} 
            width={600} 
            height={450} 
            style={{ width: '100%', height: '100%', border: '1px solid #ddd', borderRadius: '8px' }}
          />
          
          <div className="legend">
            <div className="legend-item">
              <div className="legend-color" style={{backgroundColor: 'rgba(255, 0, 0, 0.8)'}}></div>
              <span>Object Detection (below threshold)</span>
            </div>
            <div className="legend-item">
              <div className="legend-color" style={{background: 'linear-gradient(to right, hsl(270, 100%, 50%), hsl(0, 100%, 50%))'}}></div>
              <span>Distance (purple = far, red = near)</span>
            </div>
            <div className="legend-item">
              <div className="legend-color" style={{backgroundColor: 'rgba(0, 0, 0, 0.2)'}}></div>
              <span>Invalid Reading</span>
            </div>
          </div>
          
          <div style={{ marginTop: '20px' }}>
            <p><strong>How to use:</strong></p>
            <ul>
              <li>Red areas indicate objects closer than the threshold (potential drone detections)</li>
              <li>Rainbow colors show distance (red = near, purple = far)</li>
              <li>Black areas indicate invalid readings (low confidence)</li>
              <li>Use the calibration feature in the main interface to set the detection threshold</li>
            </ul>
          </div>
        </div>
      )}
    </div>
  );
};

// Initialize React
try {
  console.log('Initializing React component...');
  const rootElement = document.getElementById('root');
  if (!rootElement) {
      throw new Error('Root element not found!');
  }
  ReactDOM.render(<ToFVisualization />, rootElement);
  console.log('React component initialized');
} catch (err) {
  console.error('Failed to initialize React:', err);
  document.getElementById('root').innerHTML = 
      `<div style="color: red;">
          Failed to initialize visualization: ${err.message}<br>
          Check browser console for details.
      </div>`;
}