import socket
import time
import json

def send_udp_message(message_dict, host="127.0.0.1", port=65435):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        message = json.dumps(message_dict)
        sock.sendto(message.encode("utf-8"), (host, port))
        print(f"Sent: {message}")
    finally:
        sock.close()


string_message = {
  "topic": "chatter",
    "msgType": "String",
    "data": {
    "data": "Hello from JSON UDP!"
    }
}
send_udp_message(string_message)
