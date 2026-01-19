import rclpy
from rclpy.node import Node
from rover2_control_interface.msg import DriveCommandMessage
from rosidl_runtime_py.utilities import get_message
from unity_udp_ros_bridge.UDPServer import UDPServer
from threading import Thread

import json
import socket
import copy
from pathlib import Path

class UDPBridgeSub(Node):

    def __init__(self):
        super().__init__('udp_bridge_sub')

        self.subscription_dict = {}




        self.message_path = Path("/home/henry/custom_msg_testing/Assets/Templates")

        self.udp_server = UDPServer('127.0.0.1', 65436, logger=self.get_logger())

        # Start server in background thread
        self.udp_thread = Thread(target=self.udp_server.start, args=(self.process_udp_message,))
        self.udp_thread.daemon = True
        self.udp_thread.start()

        self.get_logger().info("UDPBridgeSub node initialized.")
        self.msg_type_cache = {}

    def set_dict_from_fields(self, msg):
        """Recursively convert ROS message to a JSON-serializable dictionary"""
        # Case 1: List or tuple of items (e.g., arrays)
        if isinstance(msg, (list, tuple)):
            return [self.set_dict_from_fields(x) for x in msg]

        # Case 2: ROS message object (has __slots__)
        elif hasattr(msg, '__slots__'):
            d = {}
            for field in msg.__slots__:
                value = getattr(msg, field)
                d[field] = self.set_dict_from_fields(value)
            return d

        # Case 3: Primitive types (float, int, str, bool, etc.)
        else:
            return msg

    def send_udp_message(self, message_dict, host="127.0.0.1", port=65435):
        """Send JSON over UDP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            message = json.dumps(message_dict)
            sock.sendto(message.encode("utf-8"), (host, port))
            self.get_logger().info(f"Sent UDP message to {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")
        finally:
            sock.close()

    def process_udp_message(self, data, addr):
        """Handle incoming UDP messages to configure subscription"""
        try:
            message = data.decode('utf-8').strip()
            print(message)
            print(type(message))
            payload = message.split(";")
            # Parse JSON
            # Expect keys: "topic" and "msgType", "data"
            topic_name = payload[0]
            messageType = payload[1]
          
            if topic_name is None or messageType is None:
                self.get_logger().error("Package missing required keys 'topic' or 'msgType'")
                return
            # Get message class dynamically
            if messageType not in self.msg_type_cache:
                self.msg_type_cache[messageType] = get_message(messageType)
                self.get_logger().info(f"Created subscriber for {topic_name} with type {messageType}")


            msg_class = self.msg_type_cache[messageType]

            self.create_subscription(
                msg_class,
                topic_name,
                self.make_callback(topic_name,messageType),
                10
            )



        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to process UDP message: {e}")


    def make_callback(self, topic_name, msgType):
        def callback(msg):
            self.process_subscription(msg, topic_name, msgType)
        return callback


    def process_subscription(self, msg, topic_name, msgType):
        payload = {}
        payload["topic"] = topic_name
        payload["msgType"] = msgType
        payload["data"] = self.set_dict_from_fields(msg)

        self.send_udp_message(payload)


def main(args=None):
    rclpy.init(args=args)
    udp_bridge_sub = UDPBridgeSub()
    rclpy.spin(udp_bridge_sub)
    udp_bridge_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
