# RT_SLAM – Real-Time SLAM Visualization

## 📖 Overview
**RT_SLAM** is a real-time **Simultaneous Localization and Mapping (SLAM) visualization tool** built using **React**.  
It connects to a WebSocket backend to display **vehicle pose updates**, **map updates**, and **system health information** in a clean and interactive dashboard.

This project is ideal for **robotics, autonomous driving, and research applications**, providing a **web-based monitoring tool** for SLAM systems.

---

## ✨ Key Features
- 🔴 **Real-time WebSocket connection** to backend  
- 🗺️ **Dynamic Map Visualization** (updates live from backend)  
- 🚗 **Vehicle Pose Tracking** (`x`, `y`, `θ`)  
- 📊 **System Health Monitoring Panel**  
- 🔄 **Configurable Trail Length** (default: 200 poses)  
- 🌐 **Modern React-based UI**  

---

## 📂 Project Structure

RT_SLAM/
│── src/
│ ├── App.js # Main React application
│ ├── App.css # Styling
│ ├── components/
│ │ ├── MapView.js # Map visualization (pose + trail + map data)
│ │ ├── SystemHealthPanel.js # Displays system health
│ ├── services/
│ │ ├── websocketService.js # WebSocket connection & handlers
│ └── index.js # React entry point
│
│── package.json # Dependencies & scripts
│── README.md # Documentation


---

## 🚀 Getting Started

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/your-username/RT_SLAM.git
cd RT_SLAM

2️⃣ Install Dependencies

npm install

3️⃣ Run the Development Server

npm start

Now open your browser at 👉 http://localhost:3000
🔌 WebSocket API Format

The frontend receives JSON messages from the backend WebSocket:
📍 Pose Update

{
  "type": "pose_update",
  "data": {
    "x": 12.34,
    "y": 56.78,
    "theta": 1.57
  }
}

🗺️ Map Update

{
  "type": "map_update",
  "data": {
    "mapGrid": [[0,1,0], [1,0,1]]
  }
}

📊 Dashboard Panels

    MapView → Displays vehicle trail + map updates

    Connection Status → Shows WebSocket connection state (CONNECTED / DISCONNECTED)

    Vehicle Pose → Displays real-time position (x, y, θ)

    System Health Panel → Shows backend/system health

📌 Future Enhancements

    Multi-robot support

    ROS2 bridge integration

    Save & export map snapshots

    3D SLAM visualization support
