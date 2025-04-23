#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PromptPublisher(Node):
    def __init__(self):
        super().__init__('llava_prompt_publisher')
        self.publisher_ = self.create_publisher(String, '/llama_input', 10)
        timer_period = 1.0  # publish once per second (optional)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.prompt_published = False

    def timer_callback(self):
        if not self.prompt_published:
            msg = String()
            msg.data = """
You are a robotic assistant controlling a UR10 manipulator.
Extract structured actions in this JSON format:
{
  "action": "<action>",
  "object": "<object>",
  "start_location": "<start_location>",
  "end_location": "<end_location>"
}

Task: {"pick up the screwdriver from the table and place it on the shelf."}
"""
            self.publisher_.publish(msg)
            self.get_logger().info('Published prompt to LLaVA')
            self.prompt_published = True  # Only publish once (optional)

def main(args=None):
    rclpy.init(args=args)
    prompt_publisher = PromptPublisher()
    rclpy.spin(prompt_publisher)
    prompt_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
