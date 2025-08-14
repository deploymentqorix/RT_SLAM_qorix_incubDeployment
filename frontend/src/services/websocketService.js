const MOCK_MODE = false;
const WEBSOCKET_URL = "ws://localhost:3000/ws";

let socket = null;
let mockInterval = null;

export const connectWebSocket = (onMessageCallback) => {
  if (MOCK_MODE) {
    console.log("Starting in MOCK mode (Map will not be displayed).");
    let step = 0;
    mockInterval = setInterval(() => {
      step += 0.05;
      const fakeData = {
        type: 'pose_update',
        timestamp: new Date().getTime(),
        data: { x: 15 * Math.cos(step), y: 10 * Math.sin(step), theta: step },
      };
      onMessageCallback(fakeData);
    }, 100);

  } else {
    if (socket && socket.readyState === WebSocket.OPEN) return;
    socket = new WebSocket(WEBSOCKET_URL);
    socket.onopen = () => console.log("WebSocket connection established.");
    socket.onmessage = (event) => {
      try {
        onMessageCallback(JSON.parse(event.data));
      } catch (error) { console.error("Error parsing WebSocket message:", error); }
    };
    socket.onerror = (error) => console.error("WebSocket Error:", error);
    socket.onclose = () => { console.log("WebSocket connection closed."); socket = null; };
  }
};

export const disconnectWebSocket = () => {
  if (MOCK_MODE) {
    if (mockInterval) clearInterval(mockInterval);
  } else {
    if (socket) socket.close();
  }
};