/* Message for Live Pose Updates */
{
  "type": "pose_update",
  "data": { "x": 10.5, "y": -4.2, "theta": 2.14 }
}

/* Message for LiDAR Point Cloud (can be downsampled) */
{
  "type": "lidar_update",
  "data": {
    "points": [ /* Array of {x, y, z} objects */ ]
  }
}

/* Message for Object Detections on Camera Feed */
{
  "type": "detection_update",
  "data": {
    "detections": [
      {"id": 1, "label": "car", "box2d": [100, 150, 50, 40]}
    ]
  }
}