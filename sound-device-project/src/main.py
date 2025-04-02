import google.generativeai as genai
import os
import sys
import json
import logging

# setup gemini and (eventually) ros2
GEMINI_API_KEY = "AIzaSyAiAWiOwZzUxxNHwwl9RF7VPkmMP77EE3c" # replace wiht your own api key
GEMINI_MODEL_NAME = "gemini-1.5-flash" 
# intended ROS topic/service names, this should probably be changed once we actually init ros2
CMD_VEL_TOPIC = "/cmd_vel"
SPEAK_TOPIC = "/speak"

PROMPT_TEMPLATE = """ """

class GeminiInterpreter:
    def __init__(self):
        # actually config gemini, this could be totally wrong dude lmfao
        try:
            genai.configure(api_key=GEMINI_API_KEY)
            self.gemini_model = genai.GenerativeModel(GEMINI_MODEL_NAME)
            logger.info(f"Gemini model '{GEMINI_MODEL_NAME}' initialized.")
        except Exception as e:
            logger.error(f"Failed to initialize Gemini: {e}")
            sys.exit(1)
    
    def call_gemini(self, user_text):
        prompt = PROMPT_TEMPLATE.format(user_input=user_text)
        
        try:
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
            print(f"Error calling gemini: {e}")
            return 

def main():
    interpreter = GeminiInterpreter()

    try:
        while True: # loop indefinitely
            try:
                user_input = input("exit to leave")
                if user_input.lower() == "exit":
                    break
                if not user_input:
                    continue

                command_json = interpreter.call_gemini(user_input) 
            
            except Exception as e: 
                 logger.error(f"problem in main loop: {e}")

    except KeyboardInterrupt:
        print("\nExiting.") # ctrl + c
    finally:
        return

if __name__ == '__main__':
    main()