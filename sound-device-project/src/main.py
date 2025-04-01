import google.generativeai as genai
import os
import sys
import json
import logging

# setup gemini and (eventually) ros2
GEMINI_API_KEY = "AIzaSyAiAWiOwZzUxxNHwwl9RF7VPkmMP77EE3c" # replace wiht your own api key
GEMINI_MODEL_NAME = "gemini-1.5-flash" # went for free model
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

def main():
    interpreter = GeminiInterpreter()

    try:
        while True: # loop indefinitely
            user_input = input("exit to leave")
            if user_input.lower() == "exit":
                break

    except KeyboardInterrupt:
        print("\nExiting.") # ctrl + c
    finally:
        return


if __name__ == '__main__':
    main()