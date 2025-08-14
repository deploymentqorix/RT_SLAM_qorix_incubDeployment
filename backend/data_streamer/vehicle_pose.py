import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber_for_streamer')
        self.subscription = self.create_subscription(
            PoseStamped,
            'vehicle_pose',  # This MUST match the topic from the C++ publisher
            self.listener_callback,
            10)
        self.get_logger().info('Data Streamer node started, listening for pose data...')

    def listener_callback(self, msg):
        """This function is called every time a message is received."""
        pos = msg.pose.position
        self.get_logger().info(f'Received Pose: x={pos.x:.2f}, y={pos.y:.2f}')
        # In the future, this is where you will send the data over the WebSocket.

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()