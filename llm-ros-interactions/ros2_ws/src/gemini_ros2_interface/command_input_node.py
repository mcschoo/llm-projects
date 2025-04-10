#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys

# NEW Topic for raw user input
PUB_TOPIC = "/raw_user_command"

class CommandInputNode(Node):
    """
    Node to capture user input from the command line and publish it RAW.
    """
    def __init__(self):
        super().__init__('command_input_node')
        # Publish to the new raw command topic
        self.publisher_ = self.create_publisher(String, PUB_TOPIC, 10)
        self.get_logger().info(f'Command Input Node started. Publishing raw commands to {PUB_TOPIC}.')

    def publish_command(self, command_text):
        """Publishes the user's raw command."""
        msg = String()
        msg.data = command_text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing raw command: "{command_text}"')

def main(args=None):
    rclpy.init(args=args)
    command_input_node = CommandInputNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(command_input_node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            try:
                user_input = input("Enter command for robot (or 'exit' to quit): ")
                if user_input.lower() == 'exit':
                    break
                if not user_input:
                    continue
                command_input_node.publish_command(user_input)
            except EOFError:
                break
            except Exception as e:
                 command_input_node.get_logger().error(f"Error in input loop: {e}")

    except KeyboardInterrupt:
        command_input_node.get_logger().info('Ctrl+C detected, shutting down.')
    finally:
        command_input_node.get_logger().info('Shutting down Command Input Node.')
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()