import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class LlamaJsonSubscriber(Node):
    def __init__(self):
        super().__init__('llama_json_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/llama_output',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Llama JSON Subscriber is up and listening to /llama_output...")

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.get_logger().info("Received JSON:")
            for key, value in data.items():
                self.get_logger().info(f"{key}: {value}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON: " + msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = LlamaJsonSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
