# Mock Sending Commands to ROS2 with Google's Gemini

## Project Requirements
    - Google AI Studio account, with an API key which you'll substitute into this project
    - Google's generativeai python library installed on your system
    - Python libs: sounddevice, numpy, & SpeechRecognition (pip install sounddevice numpy SpeechRecognition)

## How to Run the Code
    python sound-device-project/cli/main.py

## About
    Currently I'm using the flash model of gemini to keep costs non-existent. While this model works and is exceptional for the size, it doesn't have a lot of context. That means you'll need to be pretty direct with the commands you enter.

    Right now I'm working on integrating python's sounddevice library into the project so you can talk to it - that's what the "sd" folder is about. Cli is the working iteration.