
import socket


class UDPServer:
    def __init__(self, host: str, port: int, logger=None):
        self.host = host
        self.port = port
        self.udp_socket = None
        self.client_address = None
        self.running = False
        self.logger = logger

    def log(self, msg: str, level="info"):
        if self.logger:
            getattr(self.logger, level)(msg)

    def start(self, callback):
        """Start UDP server loop in this thread"""
        try:
            self.log(f"Creating UDP socket...")
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind((self.host, self.port))
            self.udp_socket.settimeout(1.0)
            
            self.log(f"UDP server bound to {self.host}:{self.port}")
            self.running = True

            while self.running:
                try:
                    data, addr = self.udp_socket.recvfrom(1024)


                    if self.client_address != addr:
                        self.client_address = addr
                        self.log(f"New client: {addr}")

                    # Call the provided callback with the raw data
                    callback(data, addr)

                except socket.timeout:
                    continue
                except Exception as e:
                    self.log(f"UDP server error: {e}", level="error")

        except Exception as e:
            self.log(f"Failed to start UDP server: {e}", level="error")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.udp_socket:
            self.log("Closing UDP socket...")
            self.udp_socket.close()
            self.udp_socket = None
