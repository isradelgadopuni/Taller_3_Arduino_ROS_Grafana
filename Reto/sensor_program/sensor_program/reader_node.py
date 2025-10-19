import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReaderNode(Node):
    def __init__(self):
        super().__init__('reader_node')
        self.sub = self.create_subscription(String, 'sensor_data', self.cb, 10)
        self.pub = self.create_publisher(String, 'reader_data', 10)  # NUEVO

    def cb(self, msg: String):
        self.get_logger().info(f'Recibido: {msg.data}')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
