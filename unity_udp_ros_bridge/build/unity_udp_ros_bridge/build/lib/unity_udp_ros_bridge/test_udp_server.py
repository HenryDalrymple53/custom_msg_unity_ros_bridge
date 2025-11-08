import socket
import json

def udp_json_receiver(host="0.0.0.0", port=65435, buffer_size=8192):
    """
    Listens for UDP JSON packets and prints them.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    print(f"Listening for UDP JSON messages on {host}:{port}...\n")

    while True:
        try:
            data, addr = sock.recvfrom(buffer_size)
            message = data.decode("utf-8").strip()

            try:
                json_data = json.loads(message)
                print(f"From {addr}:")
                print(json.dumps(json_data, indent=2))
            except json.JSONDecodeError:
                print(f"Invalid JSON from {addr}: {message}")

            print("-" * 60)

        except KeyboardInterrupt:
            print("\nStopping receiver.")
            break

    sock.close()


if __name__ == "__main__":
    udp_json_receiver()
