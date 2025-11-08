import rclpy
from rclpy.node import Node
from rover2_control_interface.msg import DriveCommandMessage
import json
import socket
import copy

class UDPBridgeSub(Node):

    def __init__(self):
        super().__init__('udp_bridge_sub')

        self.subscription = self.create_subscription(
            DriveCommandMessage,
            'drive_topic',
            self.process_drive_subscribe,
            10
        )

        self.default_json = {
            "topic": "",
            "msgType": "",
            "data": {}
        }

        self.get_logger().info("UDPBridgeSub node initialized.")
        self.subscription  # prevent unused variable warning

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

    def process_drive_subscribe(self, msg):
        """Convert DriveCommandMessage → JSON → Send over UDP"""
        payload = copy.deepcopy(self.default_json)
        payload["topic"] = "drive_topic"
        payload["msgType"] = "DriveCommandMessage"
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
