#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys

# All this does is capture a command sent by the user through the cli, and then publishes it to any of it's subscribers
class CommandInputNode(Node):
    def __init__(self):
        super().__init__('command_input_node')
        self.publisher_ = self.create_publisher(String, '/user_command_text', 10)
        self.get_logger().info('Command Input Node started. Ready for input.')

    def publish_command(self, command_text):
        msg = String()
        msg.data = command_text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: "{command_text}"')

def main(args=None):
    rclpy.init(args=args)
    command_input_node = CommandInputNode()

    # Spin the node in a separate thread to avoid blocking the input loop
    spin_thread = threading.Thread(target=rclpy.spin, args=(command_input_node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            try:
                # Get input from the user in the main thread
                user_input = input("Enter command for robot (or 'exit' to quit): ")
                if user_input.lower() == 'exit':
                    break
                if not user_input:
                    continue
                # Publish the command using the node's method
                command_input_node.publish_command(user_input)
            except EOFError: # Handle Ctrl+D
                break
            except Exception as e:
                 command_input_node.get_logger().error(f"Error in input loop: {e}")

    except KeyboardInterrupt:
        command_input_node.get_logger().info('Ctrl+C detected, shutting down.')
    finally:
        command_input_node.get_logger().info('Shutting down Command Input Node.')
        rclpy.shutdown()
        spin_thread.join() # Wait for spin thread to finish cleanly

if __name__ == '__main__':
    main()