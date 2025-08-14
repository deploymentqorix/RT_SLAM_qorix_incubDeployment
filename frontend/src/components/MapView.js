import React, { useRef, useEffect } from 'react';

const MapView = ({ pose, trail, mapData }) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // --- Clear and prepare canvas ---
    ctx.fillStyle = '#1e1e1e';
    ctx.fillRect(0, 0, width, height);

    // --- Draw Map Data (if available) ---
    if (mapData) {
      const { resolution, width: mapWidth, height: mapHeight, origin, grid_data } = mapData;
      const imageData = ctx.createImageData(mapWidth, mapHeight);
      
      for (let i = 0; i < grid_data.length; i++) {
        const value = grid_data[i];
        let color;
        if (value === 100) { // Occupied
          color = [40, 40, 40, 255]; 
        } else if (value === 0) { // Free
          color = [100, 100, 100, 255];
        } else { // Unknown
          color = [60, 60, 60, 255];
        }
        imageData.data[i * 4] = color[0];
        imageData.data[i * 4 + 1] = color[1];
        imageData.data[i * 4 + 2] = color[2];
        imageData.data[i * 4 + 3] = color[3];
      }
      
      const mapCanvas = document.createElement('canvas');
      mapCanvas.width = mapWidth;
      mapCanvas.height = mapHeight;
      mapCanvas.getContext('2d').putImageData(imageData, 0, 0);

      ctx.save();
      ctx.translate(width / 2, height / 2);
      const mapScale = 3.0;
      ctx.scale(mapScale, mapScale);
      // Center the map based on its origin
      ctx.translate(origin.x / resolution, -origin.y / resolution);
      ctx.drawImage(mapCanvas, 0, 0, mapWidth, mapHeight);
      ctx.restore();
    }

    // --- Drawing Vehicle and Trail ---
    const vehicleScale = 10; // <-- REDUCED SCALE for vehicle and trail
    
    // --- Draw Trail ---
    if (trail.length > 1) {
      ctx.strokeStyle = '#f0a050';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(width / 2 + trail[0].x * vehicleScale, height / 2 - trail[0].y * vehicleScale);
      for (let i = 1; i < trail.length; i++) {
        ctx.lineTo(width / 2 + trail[i].x * vehicleScale, height / 2 - trail[i].y * vehicleScale);
      }
      ctx.stroke();
    }

    // --- Draw Vehicle ---
    const vehicleSize = 8;
    const mapX = width / 2 + pose.x * vehicleScale;
    const mapY = height / 2 - pose.y * vehicleScale;
    
    ctx.save();
    ctx.translate(mapX, mapY);
    ctx.rotate(-pose.theta);
    ctx.beginPath();
    ctx.moveTo(vehicleSize, 0);
    ctx.lineTo(-vehicleSize / 2, -vehicleSize / 2);
    ctx.lineTo(-vehicleSize / 2, vehicleSize / 2);
    ctx.closePath();
    ctx.fillStyle = '#61dafb';
    ctx.fill();
    ctx.restore();

  }, [pose, trail, mapData]);

  return <canvas ref={canvasRef} width={600} height={600} />;
};

export default MapView;
