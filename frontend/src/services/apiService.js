const API_URL = "http://localhost:8000";
const MOCK_MODE = true; // Uses mock data from websocketService

export const fetchSystemHealth = async () => {
  if (MOCK_MODE) {
    // --- MOCK MODE: Return fake data ---
    // Simulate a network delay
    await new Promise(resolve => setTimeout(resolve, 500)); 
    
    // Return a fake health status object
    return {
      cpu_usage: 35.4 + Math.random() * 10, // Add some randomness
      memory: { used: 4.1 + Math.random(), total: 16.0 },
      slam_status: "ACTIVE",
    };
  } else {
    // --- REAL MODE: Fetch from the backend ---
    try {
      const response = await fetch(`${API_URL}/api/v1/system/health`);
      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error("Could not fetch system health:", error);
      // Return a default error state
      return {
        cpu_usage: 0,
        memory: { used: 0, total: 0 },
        slam_status: "ERROR",
      };
    }
  }
};