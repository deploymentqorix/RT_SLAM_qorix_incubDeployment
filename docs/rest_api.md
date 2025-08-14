# REST API Endpoints

## Get System Health
GET /api/v1/system/health
--> Response 200 OK:
{
  "cpu_usage": 45.5,
  "memory": {"used": 4.1, "total": 8.0},
  "gpu_usage": 60.1,
  "slam_status": "ACTIVE"
}

## Get Last 20 Alerts
GET /api/v1/alerts?limit=20
--> Response 200 OK:
{
  "alerts": [
    {"timestamp": "...", "severity": "WARN", "message": "IMU connection lost"},
    {"timestamp": "...", "severity": "INFO", "message": "New object detected"}
  ]
}

## Reset SLAM System
POST /api/v1/slam/reset
--> Response 200 OK:
{ "status": "ok", "message": "SLAM reset command received" }
