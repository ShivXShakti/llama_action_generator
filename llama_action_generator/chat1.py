import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_cpp import Llama
import json
import re

class LlamaNode(Node):
    def __init__(self):
        super().__init__('llama_node')

        # Set up subscriber and publisher
        self.subscription = self.create_subscription(String, '/llama_input', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, '/llama_output', 10)

        # Load the model
        self.get_logger().info("Loading TinyLLaMA model...")
        self.llm = Llama.from_pretrained(
	            repo_id="ybelkada/TinyLlama-1.1B-Chat-v1.0-Q4_0-GGUF",
	            filename="tinyllama-1.1b-chat-v1.0.Q4_0.gguf",)
        self.get_logger().info("Model loaded successfully!")

    def format_to_json(self,raw_output):
        # Convert lines like: Action: "pick up screwdriver" into JSON key-value pairs
        lines = raw_output.splitlines()
        json_dict = {}
        for line in lines:
            match = re.match(r'(\w[\w\s]*):\s*"(.*)"', line)
            if match:
                key = match.group(1).strip().lower().replace(" ", "_")  # e.g., "Start location" → "start_location"
                value = match.group(2).strip()
                json_dict[key] = value
        return json_dict
    
    def listener_callback(self, msg):
        prompt = """
You are a robotic assistant controlling a UR10 manipulator.

Extract structured actions in this exact JSON format:
{
  "action": "<action>",
  "object": "<object>",
  "start_location": "<start_location>",
  "end_location": "<end_location>"
}

Respond with a single JSON object. DO NOT include any explanation, extra text, or multiple examples.

If the object is not found in the image, return:
{
  "action": "no",
  "object": "none",
  "start_location": "none",
  "end_location": "none"
}

Task: pick up the screwdriver from the table and place it on the shelf.
"""
        #self.get_logger().info(f"Received prompt: {prompt}")

        response = self.llm(prompt, max_tokens=256, stop=["\n\n"], echo=False)
        output = response["choices"][0]["text"].strip()
        self.get_logger().info(f"Raw response: {output}")

        try:
            parsed = json.loads(output)
        except json.JSONDecodeError:
            self.get_logger().warn("Trying to convert non-JSON formatted output.")
            parsed = self.format_to_json(output)

        if not parsed:
            self.get_logger().error(f"Failed to parse or convert output: {output}")
            return

        action = parsed.get("action", "")
        obj = parsed.get("object", "")
        start_location = parsed.get("start_location", "")
        end_location = parsed.get("end_location", "")

        self.get_logger().info(f"Action: {action}")
        self.get_logger().info(f"Object: {obj}")
        self.get_logger().info(f"Start Location: {start_location}")
        self.get_logger().info(f"End Location: {end_location}")

        
        # Convert to compact JSON string
        json_output = json.dumps(parsed)

        out_msg = String()
        out_msg.data = json_output
        self.publisher.publish(out_msg)
        self.get_logger().info(f"Published structured JSON: {json_output}")

def main(args=None):
    rclpy.init(args=args)
    node = LlamaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
