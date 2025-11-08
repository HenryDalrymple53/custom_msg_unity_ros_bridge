import rclpy
from rclpy.node import Node
from unity_udp_ros_bridge.UDPServer import UDPServer
from threading import Thread

from std_msgs.msg import String
from rover2_control_interface.msg import DriveCommandMessage
import json

class UDPBridgePub(Node):
    """ROS2 Node using JSON over UDP"""

    def __init__(self):
        super().__init__('udp_bridge_pub')

        # Dictionary: key = topic_name, value = publisher object
        self.publisher_dict = {}

        # Create UDP server instance
        self.udp_server = UDPServer('127.0.0.1', 65434, logger=self.get_logger())

        # Start server in background thread
        self.udp_thread = Thread(target=self.udp_server.start, args=(self.process_udp_message,))
        self.udp_thread.daemon = True
        self.udp_thread.start()

        self.get_logger().info("UDPBridgePub node initialized.")

    def set_fields_from_dict(self, msg_obj, data_dict):
        
        #set message fields from a nested dictionary
        
        for key, value in data_dict.items():
            if hasattr(msg_obj, key):
                attr = getattr(msg_obj, key)
                if hasattr(attr, '__slots__') and isinstance(value, dict):
                    self.set_fields_from_dict(attr, value)
                else:
                    field_type = type(attr)
                    setattr(msg_obj, key, field_type(value))

            else:
                self.get_logger().warning(f"Message {type(msg_obj).__name__} has no field '{key}'")

    def process_udp_message(self, data, addr):
        """Handle incoming UDP messages as JSON"""
        try:
            message = data.decode('utf-8').strip()

            # Parse JSON
            payload = json.loads(message)

            # Expect keys: "topic" and "msgType", "data"
            topic_name = payload.get("topic")
            messageType = payload.get("msgType")
          
            data_dict = payload.get("data", {})

            if topic_name is None or messageType is None:
                self.get_logger().error("JSON missing required keys 'topic' or 'msgType'")
                return

            # Get message class dynamically
            if messageType not in globals():
                self.get_logger().error(f"Unknown message type: {messageType}")
                return
            msg_class = globals()[messageType]

            # Create publisher if it doesn't exist
            if topic_name not in self.publisher_dict:
                self.publisher_dict[topic_name] = self.create_publisher(msg_class, topic_name, 10)
                self.get_logger().info(f"Created publisher for {topic_name} with type {messageType}")

            # Create new message instance
            ros2_msg = msg_class()
            self.set_fields_from_dict(ros2_msg, data_dict)

            # Publish the message
            self.publisher_dict[topic_name].publish(ros2_msg)
            self.get_logger().info(f"Published message on {topic_name}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to process UDP message: {e}")

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info("Shutting down node...")
        self.udp_server.stop()
        if self.udp_thread.is_alive():
            self.udp_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = UDPBridgePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
