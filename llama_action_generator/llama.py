from llama_cpp import Llama

# Load LLaMA model
llm = Llama.from_pretrained(
	repo_id="ybelkada/TinyLlama-1.1B-Chat-v1.0-Q4_0-GGUF",
	filename="tinyllama-1.1b-chat-v1.0.Q4_0.gguf",
)

# Prompt as above
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

# Inference
response = llm(prompt, max_tokens=256, stop=["\n\n"], echo=False)

# Extract text
output = response["choices"][0]["text"].strip()

# Print or use output
print(output)
