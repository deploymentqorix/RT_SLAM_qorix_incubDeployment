import React, { useState, useEffect } from 'react';
import { fetchSystemHealth } from '../services/apiService';

const SystemHealthPanel = () => {
  const [health, setHealth] = useState({
    cpu_usage: 0,
    memory: { used: 0, total: 0 },
    slam_status: "LOADING...",
  });

  useEffect(() => {
    // Fetch data every 2 seconds
    const interval = setInterval(async () => {
      const data = await fetchSystemHealth();
      setHealth(data);
    }, 2000); // 2000ms = 2 seconds

    // Clean up the interval when the component is removed
    return () => clearInterval(interval);
  }, []);

  const memoryUsage = health.memory.total > 0 
    ? (health.memory.used / health.memory.total * 100).toFixed(1) 
    : 0;

  return (
    <div className="panel">
      <h2>System Health</h2>
      <p><strong>SLAM Status:</strong> 
        <span className={health.slam_status === 'ACTIVE' ? 'status-ok' : 'status-error'}>
          {health.slam_status}
        </span>
      </p>
      <p><strong>CPU Usage:</strong> {health.cpu_usage.toFixed(1)}%</p>
      <p><strong>Memory:</strong> {health.memory.used.toFixed(1)} / {health.memory.total.toFixed(1)} GB ({memoryUsage}%)</p>
    </div>
  );
};

export default SystemHealthPanel;
