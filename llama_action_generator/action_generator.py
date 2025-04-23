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
	            filename="tinyllama-1.1b-chat-v1.0.Q4_0.gguf",
                chat_format="chatml")
        self.get_logger().info("Model loaded successfully!")
    
    def extract_first_json(self,text):
        json_blocks = re.findall(r'\{[^{}]+\}', text, re.DOTALL)
        for jb in json_blocks:
            try:
                return json.loads(jb)
            except json.JSONDecodeError:
                continue
        return None

    def listener_callback(self, msg):

        response = self.llm.create_chat_completion(
        messages=[
        {"role": "system", "content": "You are a robotic assistant controlling a UR10 manipulator."},
        {"role": "user", "content": """Extract structured actions in this exact JSON format:
        {
        "action": "<action>",
        "object": "<object>",
        "start_location": "<start_location>",
        "end_location": "<end_location>"
        }

        Respond with a single JSON object. DO NOT include any explanation, extra text, or multiple examples.

        Task: pick up the remote from the ground and place it in red container."""}
        ],
        max_tokens=256
        )
        #Task: pick up the screwbdriver from the table and place it on the shelf.
        output = response["choices"][0]["message"]["content"].strip()
        self.get_logger().info(f"Raw output: {output}")

        parsed = self.extract_first_json(output)
        if not parsed:
            self.get_logger().error(f"Failed to extract valid JSON from: {output}")
            return
        json_output = json.dumps(parsed)
        out_msg = String()
        out_msg.data = json_output
        self.publisher.publish(out_msg)
        self.get_logger().info(f"Published structured JSON: {json_output}")
        
        action = parsed.get("action", "")
        obj = parsed.get("object", "")
        start_location = parsed.get("start_location", "")
        end_location = parsed.get("end_location", "")

        self.get_logger().info(f"Action: {action}")
        self.get_logger().info(f"Object: {obj}")
        self.get_logger().info(f"Start Location: {start_location}")
        self.get_logger().info(f"End Location: {end_location}") 

def main(args=None):
    rclpy.init(args=args)
    node = LlamaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
