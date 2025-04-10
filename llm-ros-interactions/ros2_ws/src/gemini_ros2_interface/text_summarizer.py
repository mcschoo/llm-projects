#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline
import sys
import torch

# Topic to listen for raw commands
SUB_TOPIC = "/raw_user_command"
# Topic to publish the summarized command to (for Gemini node)
PUB_TOPIC = "/user_command_text"

class SummarizerNode(Node): # Kept original class name, node name changed below
    def __init__(self):
        # Renamed node to be more specific about its processing role
        super().__init__('summarizer_processor_node')

        # Subscribe to the RAW command text
        self.subscription = self.create_subscription(
            String,
            SUB_TOPIC,
            self.listener_callback, # Callback processes the raw text
            10
        )
        self.subscription # prevent unused variable warning

        # Publisher sends summarized results to the Gemini node
        self.publisher_ = self.create_publisher(
            String,
            PUB_TOPIC,
            10
        )

        # Initialize the summarizer pipeline
        try:
            device = 0 if torch.cuda.is_available() else -1
            self.get_logger().info(f"Using device {'GPU' if device == 0 else 'CPU'} for summarization.")
            self.summarizer = pipeline("summarization", model="facebook/bart-large-cnn", device=device)
            self.get_logger().info("Summarization pipeline initialized.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize summarization pipeline: {e}")
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info(f"Summarizer Processor Node started. Listening on {SUB_TOPIC}, Publishing to {PUB_TOPIC}.")

    def summarise_text(self, text):
        """Summarizes the input text."""
        if not text:
            self.get_logger().warn("Received empty text, nothing to summarize.")
            return None # Return None if no text

        self.get_logger().info(f"Summarizing: '{text}'")
        try:
            # Summarize (adjust lengths as needed)
            summary_result = self.summarizer(text, max_length=60, min_length=5, do_sample=False)
            summary_text = summary_result[0]['summary_text']
            return summary_text
        except Exception as e:
            self.get_logger().error(f"Error during summarization: {e}")
            return None # Return None on error

    def listener_callback(self, msg):
        """Callback function called when raw command is received."""
        input_text = msg.data
        self.get_logger().info(f"Received raw command: '{input_text}'")

        summary = self.summarise_text(input_text)

        if summary is not None: # Only publish if summarization was successful
            # Publish the summary
            pub_msg = String()
            pub_msg.data = summary
            self.publisher_.publish(pub_msg)
            self.get_logger().info(f"Published summary: '{summary}'")
        else:
            self.get_logger().warn("Summarization failed or produced no output. Not publishing.")


def main(args=None):
    rclpy.init(args=args)
    summarizer_node = SummarizerNode() # Create instance
    try:
        rclpy.spin(summarizer_node) # Keep node alive to receive messages
    except KeyboardInterrupt:
         summarizer_node.get_logger().info('Ctrl+C detected, shutting down.')
    finally:
        summarizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()