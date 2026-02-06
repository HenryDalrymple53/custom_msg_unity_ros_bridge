import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class SetBoolServer(Node):
    def __init__(self):
        super().__init__('setbool_server')
        self.create_service(SetBool, '/test_srv', self.callback)

    def callback(self, request, response):
        self.get_logger().info(f"Received from Unity: {request.data}")
        response.success = True
        response.message = "Acknowledged"
        return response

def main():
    rclpy.init()
    node = SetBoolServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
