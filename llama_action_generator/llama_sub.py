import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_cpp import Llama
import json

class LlamaNode(Node):
    def __init__(self):
        super().__init__('llama_node')

        self.subscription = self.create_subscription(String, '/llama_output', self.listener_callback, 10)
        self.llama_action = None

    def listener_callback(self, msg):
        self.llama_action = msg.data
        try:
            action_dict = json.loads(self.llama_action)

            action = action_dict.get("action", "")
            obj = action_dict.get("object", "")
            start_location = action_dict.get("start_location", "")
            end_location = action_dict.get("end_location", "")
            if obj in ["tv", "bottle", "glass"]:
                print(f"Working... in list")
            else:
                print(f"didn't work...")

            print(f"Action: {type(action)}")
            print(f"Object: {obj}")
            print(f"Start Location: {start_location}")
            print(f"End Location: {end_location}")

        except json.JSONDecodeError as e:
            print(f"Failed to parse JSON: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LlamaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
