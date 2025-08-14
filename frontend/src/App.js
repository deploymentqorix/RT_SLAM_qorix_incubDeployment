import React, { useState, useEffect } from 'react';
import { connectWebSocket, disconnectWebSocket } from './services/websocketService';
import MapView from './components/MapView';
import SystemHealthPanel from './components/SystemHealthPanel';
import './App.css';

const MAX_TRAIL_LENGTH = 200;

function App() {
  const [isConnected, setIsConnected] = useState(false);
  const [pose, setPose] = useState({ x: 0, y: 0, theta: 0 });
  const [trail, setTrail] = useState([]);
  const [mapData, setMapData] = useState(null); // <-- NEW STATE FOR MAP

  useEffect(() => {
    const handleWebSocketMessage = (data) => {
      setIsConnected(true); // Assume connected if any message arrives
      if (data.type === 'pose_update') {
        const newPose = data.data;
        setPose(newPose);
        setTrail(prevTrail => {
          const updatedTrail = [...prevTrail, newPose];
          return updatedTrail.length > MAX_TRAIL_LENGTH
            ? updatedTrail.slice(updatedTrail.length - MAX_TRAIL_LENGTH)
            : updatedTrail;
        });
      } else if (data.type === 'map_update') {
        // Handle new map data
        setMapData(data.data);
      }
    };

    connectWebSocket(handleWebSocketMessage);
    return () => disconnectWebSocket();
  }, []);

  return (
    <div className="App">
      <header className="App-header">
        <h1>Real-Time SLAM Visualization</h1>
      </header>
      <main className="main-content">
        <div className="map-container">
          {/* Pass the map data down to the MapView component */}
          <MapView pose={pose} trail={trail} mapData={mapData} />
        </div>
        <div className="sidebar">
          <div className="panel">
            <h2>Connection Status</h2>
            <p className={isConnected ? 'connected' : 'disconnected'}>
              {isConnected ? '● CONNECTED' : '● DISCONNECTED'}
            </p>
          </div>
          <div className="panel">
            <h2>Live Vehicle Pose</h2>
            <p><strong>X:</strong> {pose.x.toFixed(2)} meters</p>
            <p><strong>Y:</strong> {pose.y.toFixed(2)} meters</p>
            <p><strong>Theta:</strong> {pose.theta.toFixed(2)} rad</p>
          </div>
          <SystemHealthPanel />
        </div>
      </main>
    </div>
  );
}

export default App;