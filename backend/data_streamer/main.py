import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import asyncio
import websockets
import json

CONNECTED_CLIENTS = set()

async def register_client(websocket):
    print("Frontend client connected!")
    CONNECTED_CLIENTS.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        print("Frontend client disconnected.")
        CONNECTED_CLIENTS.remove(websocket)

async def broadcast_message(message):
    if CONNECTED_CLIENTS:
        await asyncio.wait([client.send(message) for client in CONNECTED_CLIENTS])

class DataStreamerNode(Node):
    def __init__(self):
        super().__init__('data_streamer_node')
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.get_logger().info('Data Streamer node started, listening for odom and map data...')

    def odom_callback(self, msg):
        pose_data = {
            'type': 'pose_update',
            'data': { 'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y, 'theta': msg.pose.pose.orientation.z }
        }
        asyncio.create_task(broadcast_message(json.dumps(pose_data)))

    def map_callback(self, msg):
        self.get_logger().info(f'Broadcasting map update ({msg.info.width}x{msg.info.height})')
        map_data = {
            'type': 'map_update',
            'data': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': { 'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y },
                'grid_data': list(msg.data)
            }
        }
        asyncio.create_task(broadcast_message(json.dumps(map_data)))

async def main_loop(ros_node):
    while rclpy.ok():
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        await asyncio.sleep(0.001)

async def main(args=None):
    rclpy.init(args=args)
    data_streamer_node = DataStreamerNode()
    server = await websockets.serve(register_client, "0.0.0.0", 8765)
    print("!!! WebSocket server started on ws://0.0.0.0:8765 !!!")
    await main_loop(data_streamer_node)
    server.close()
    await server.wait_closed()
    data_streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
