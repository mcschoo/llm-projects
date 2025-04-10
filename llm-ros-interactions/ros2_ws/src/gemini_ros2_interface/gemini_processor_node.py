#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import os
import sys
import json
# Removed standard logging, will use ROS logger

# --- Configuration ---
# IMPORTANT: Set API Key via environment variable: export GOOGLE_API_KEY="..."
GEMINI_API_KEY = "AIzaSyAiAWiOwZzUxxNHwwl9RF7VPkmMP77EE3c"
GEMINI_MODEL_NAME = "gemini-1.5-flash"
# intended ROS topic/service names
CMD_VEL_TOPIC = "/cmd_vel"
SPEAK_TOPIC = "/speak" # Keep for print_intended_command context

# Subscription topic
SUB_TOPIC = "/user_command_text"
# Publication topic for processed command
PUB_TOPIC = "/gemini_ros_command" # Topic for the JSON output

PROMPT_TEMPLATE = """
You are an AI assistant translating natural language commands for a ROS2 robot.
Translate the user's command into a structured JSON format representing a specific action.

Available Actions and their JSON format:

1.  **move**: Control linear and angular velocity.
    Parameters:
    - linear_x: Forward/backward speed (m/s, typically -5 to 5)
    - angular_z: Turning speed (rad/s, typically -1.0 to 1.0)
    Example JSON: {{"action": "move", "parameters": {{"linear_x": 0.2, "angular_z": 0.0}}}}

2.  **stop**: Stop all movement.
    Parameters: None
    Example JSON: {{"action": "stop", "parameters": {{}}}}

Constraints:
- Only output a single, valid JSON object matching one of the defined actions.
- If the command is unclear, ambiguous, or requests an unsupported action, output:
  {{"action": "error", "parameters": {{"message": "Command unclear or unsupported."}}}}
- Infer reasonable parameters if not explicitly stated (e.g., "go forward" might mean linear_x=0.2).

User Command:
"{user_input}"

JSON Output:
"""

class GeminiInterpreter:
    def __init__(self, logger):
        self.logger = logger # Use the node's logger
        self.gemini_model = None
        if not GEMINI_API_KEY:
            self.logger.error("GOOGLE_API_KEY environment variable not set!")
            # Don't exit here, let the node run but Gemini calls will fail
            return

        try:
            genai.configure(api_key=GEMINI_API_KEY)
            self.gemini_model = genai.GenerativeModel(GEMINI_MODEL_NAME)
            self.logger.info(f"Gemini model '{GEMINI_MODEL_NAME}' initialized.")
        except Exception as e:
            self.logger.error(f"Failed to initialize Gemini: {e}")
            self.gemini_model = None # Ensure model is None if init fails

    def call_gemini(self, user_text):
        if not self.gemini_model:
             self.logger.error("Gemini model not initialized. Cannot process command.")
             return {"action": "error", "parameters": {"message": "Gemini not configured."}}

        if not user_text:
             return {"action": "error", "parameters": {"message": "No text provided to interpret."}}

        self.logger.info(f"Sending to Gemini: '{user_text}'")
        prompt = PROMPT_TEMPLATE.format(user_input=user_text)

        try:
            safety_settings = [
                {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
            ]
            response = self.gemini_model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    candidate_count=1,
                    temperature=0.3
                ),
                safety_settings=safety_settings
            )

            if not response.candidates:
                 self.logger.warning(f"Gemini response blocked or empty. Reason: {response.prompt_feedback}")
                 return {"action": "error", "parameters": {"message": "Gemini response blocked or empty."}}

            raw_json = response.text.strip()
            self.logger.info(f"Gemini Raw Response: {raw_json}")

            if raw_json.startswith("```json"):
                raw_json = raw_json[7:]
            if raw_json.endswith("```"):
                raw_json = raw_json[:-3]
            raw_json = raw_json.strip()

            try:
                parsed_response = json.loads(raw_json)
                if "action" not in parsed_response or "parameters" not in parsed_response:
                     raise ValueError("Missing 'action' or 'parameters' in JSON")
                return parsed_response
            except json.JSONDecodeError as e:
                self.logger.error(f"Failed to parse Gemini JSON response: {e}")
                self.logger.error(f"Raw response was: {raw_json}")
                return {"action": "error", "parameters": {"message": "Invalid JSON format from Gemini."}}
            except ValueError as e:
                 self.logger.error(f"Invalid JSON structure: {e}")
                 return {"action": "error", "parameters": {"message": f"Invalid JSON structure: {e}"}}

        except Exception as e:
            self.logger.error(f"Error calling Gemini API: {e}")
            return {"action": "error", "parameters": {"message": f"Gemini API error: {e}"}}

    # where we print the interpretation of the command, the actual action that should be taken in ROS
    def print_intended_command(self, command_data):
        if not command_data:
            self.logger.warning("No command data received to print.")
            return

        action = command_data.get("action")
        params = command_data.get("parameters", {})

        # Use logger instead of print for ROS nodes
        self.logger.info("-" * 20)

        if action == "move":
            linear_x = float(params.get("linear_x", 0.0))
            angular_z = float(params.get("angular_z", 0.0))
            self.logger.info(f"INTENDED ROS ACTION:")
            self.logger.info(f"  Publish to Topic: {CMD_VEL_TOPIC}")
            self.logger.info(f"  Message Type: geometry_msgs/msg/Twist") # Example type
            self.logger.info(f"  Data: linear.x = {linear_x:.2f}, angular.z = {angular_z:.2f}")

        elif action == "stop":
            self.logger.info(f"INTENDED ROS ACTION:")
            self.logger.info(f"  Publish to Topic: {CMD_VEL_TOPIC}")
            self.logger.info(f"  Message Type: geometry_msgs/msg/Twist") # Example type
            self.logger.info(f"  Data: linear.x = 0.0, angular.z = 0.0")

        elif action == "error":
            self.logger.error(f"ERROR from Gemini Interpretation:") # Log errors
            self.logger.error(f"  Message: {params.get('message', 'Unknown error')}")

        else:
            self.logger.warn(f"Unsupported action received: '{action}'") # Log warnings
            self.logger.warn(f"  Parameters: {params}")

        self.logger.info("-" * 20 + "\n")

# This node subscribers to the universal command node, and processes the commands with Gemini
# we're currently printing the gemini return to the cli for debugging
class GeminiProcessorNode(Node):
    def __init__(self):
        super().__init__('gemini_processor_node')
        self.interpreter = GeminiInterpreter(self.get_logger()) # Pass logger

        # Publisher for the processed JSON command
        self.publisher_ = self.create_publisher(String, PUB_TOPIC, 10)

        # Subscriber to the raw user command text
        self.subscription = self.create_subscription(
            String,
            SUB_TOPIC,
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Gemini Processor Node started. Listening on {SUB_TOPIC}.')

    # callback function to process received user commands
    def command_callback(self, msg): 
        user_text = msg.data
        self.get_logger().info(f'Received command: "{user_text}"')

        # Process with Gemini
        command_json = self.interpreter.call_gemini(user_text)

        if command_json:
            # Print the interpretation of the command
            self.interpreter.print_intended_command(command_json)

            # Publish the resulting JSON command object as a string
            try:
                json_string = json.dumps(command_json)
                pub_msg = String()
                pub_msg.data = json_string
                self.publisher_.publish(pub_msg)
                self.get_logger().info(f'Published processed command to {PUB_TOPIC}')
            except TypeError as e:
                 self.get_logger().error(f"Could not serialize command_json to string: {e}")
            except Exception as e:
                 self.get_logger().error(f"Unexpected error during JSON publishing: {e}")
        else:
            self.get_logger().warn("No valid command JSON received from Gemini to publish.")


def main(args=None):
    rclpy.init(args=args)
    gemini_processor_node = GeminiProcessorNode()
    try:
        rclpy.spin(gemini_processor_node)
    except KeyboardInterrupt:
        gemini_processor_node.get_logger().info('Ctrl+C detected, shutting down.')
    finally:
        # Destroy the node explicitly - it will be done automatically when the garbage collector destroys the node object, but this is cleaner
        gemini_processor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()