import google.generativeai as genai
import os
import sys
import json
import logging
import sounddevice as sd

# setup gemini and (eventually) ros2
GEMINI_API_KEY = "add your own api key"
GEMINI_MODEL_NAME = "gemini-1.5-flash" 
# intended ROS topic/service names, this should probably be changed once we actually init ros2
CMD_VEL_TOPIC = "/cmd_vel"
SPEAK_TOPIC = "/speak"

# config logger, replace with ros one later
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

PROMPT_TEMPLATE = """
You are an AI assistant translating natural language commands for a ROS2 robot.
Translate the user's command into a structured JSON format representing a specific action.

Available Actions and their JSON format:

1.  **move**: Control linear and angular velocity.
    Parameters:
    - linear_x: Forward/backward speed (m/s, typically -0.5 to 0.5)
    - angular_z: Turning speed (rad/s, typically -1.0 to 1.0)
    Example JSON: {{"action": "move", "parameters": {{"linear_x": 0.2, "angular_z": 0.0}}}}

2.  **stop**: Stop all movement.
    Parameters: None
    Example JSON: {{"action": "stop", "parameters": {{}}}}

# Add more actions relevant to your robot (e.g., arm control, service calls)

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
    def __init__(self):
        # actually config gemini, this could be totally wrong dude lmfao
        try:
            genai.configure(api_key=GEMINI_API_KEY)
            self.gemini_model = genai.GenerativeModel(GEMINI_MODEL_NAME)
        except Exception as e:
            logger.error(f"Failed to initialize Gemini: {e}") # debugging
            sys.exit(1)
    
    def call_gemini(self, user_text):
        prompt = PROMPT_TEMPLATE.format(user_input=user_text)
        
        try:
            safety_settings = [
                {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
                {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_MEDIUM_AND_ABOVE"},
            ]
            response = self.gemini_model.generate_content(
                prompt, # send the prompt
                generation_config=genai.types.GenerationConfig(
                    candidate_count=1,
                    temperature=0.3 # keeping this extremely deterministic
                ),
                safety_settings=safety_settings
            )

            # extract and parse the json response
            raw_json = response.text.strip()
            logger.info(f"Gemini Raw Response: {raw_json}") # debugging

            # clean markdown formatting
            if raw_json.startswith("```json"):
                raw_json = raw_json[7:]
            if raw_json.endswith("```"):
                raw_json = raw_json[:-3]
            raw_json = raw_json.strip()

            try:
                parsed_response = json.loads(raw_json)
                return parsed_response

            except json.JSONDecodeError:
                return {"action": "error", "parameters": {"message": "Invalid JSON format from Gemini."}}
            except ValueError as e:
                 return {"action": "error", "parameters": {"message": f"Invalid JSON structure: {e}"}}

        except Exception as e:
            print(f"Error calling gemini:\n{e}")
            return 

    # return command to user (what would be sent to ros)
    def print_intended_command(self, command_data):
        action = command_data.get("action")
        params = command_data.get("parameters", {})

        print("-" * 20) 

        if action == "move":
            linear_x = float(params.get("linear_x", 0.0))
            angular_z = float(params.get("angular_z", 0.0))
            print(f"INTENDED ROS ACTION:")
            print(f"  Publish to Topic: {CMD_VEL_TOPIC}")
            print(f"  Data: linear.x = {linear_x}, angular.z = {angular_z}")

        elif action == "stop":
            print(f"INTENDED ROS ACTION:")
            print(f"  Publish to Topic: {CMD_VEL_TOPIC}")
            print(f"  Data: linear.x = 0.0, angular.z = 0.0")

        elif action == "error":
            print(f"ERROR from Gemini Interpretation:")
            print(f"  Message: {params.get('message', 'Unknown error')}")

        else:
            print(f"INTENDED ROS ACTION: None (unsupported action '{action}')")

        print("-" * 20 + "\n")


def show_devices():
    print("Available Input Devices:")
    devices = sd.query_devices()
    # default_samplerate = 44100 # common default
    logDevices = []

    for i, device in enumerate(devices):
        # *****check host API to filter out some virtual/loopback devices if needed
        if device['max_input_channels'] > 0:
            # default_samplerate = int(sd.query_hostapis(device['hostapi'])['default_samplerate']) # figure out how to configure host api
            print(f"  {i}: {device['name']} (Default Sample Rate: {device['default_samplerate']} Hz)")
            # print(f"  {i}: {device['name']} (Sample Rate: {default_samplerate} Hz)")
            logDevices.append({"index": i, "samplerate": device['default_samplerate']}) #store idx + rate

    if not logDevices:
        print("  No input devices found!")
    return logDevices

def select_device(logDevices):
    if not logDevices:
        return
    
    prompt_choice = input("Choose your character: \n") # select index of device u want
    choice = int(prompt_choice)

    # "parse" choice
    get_device = next((d for d in input_devices if d['index'] == choice))
    if get_device:
        print(f"Using device {choice}.")
        return choice # return idx


def main():
    interpreter = GeminiInterpreter()

    list_devices = show_devices()
    idx = select_device(list_devices)

    try:
        while True: # loop indefinitely
            try:
                user_input = input("Send exit to leave: ")
                if user_input.lower() == "exit":
                    break
                if not user_input:
                    continue

                command_json = interpreter.call_gemini(user_input) 
                if command_json:
                    interpreter.print_intended_command(command_json)
            
            except Exception as e: 
                 logger.error(f"problem in main loop: {e}")

    except KeyboardInterrupt:
        print("\nExiting.") # ctrl + c
    finally:
        return

if __name__ == '__main__':
    main()