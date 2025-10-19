import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt

SAVE_PATH = '/ros2_ws_shared/sensor_plot.png'

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.sub = self.create_subscription(String, 'reader_data', self.cb, 10) 
        self.data = []
        self.last_save = time.time()

    def cb(self, msg: String):
        try:
            temp = float(msg.data.split(':')[1].split()[0])
            self.data.append(temp)
        except Exception:
            self.get_logger().warn(f'No pude parsear: "{msg.data}"')
            return

        if time.time() - self.last_save >= 5:
            plt.figure()
            plt.plot(self.data)
            plt.title('Temperatura simulada')
            plt.xlabel('Muestras')
            plt.ylabel('°C')
            plt.savefig(SAVE_PATH)
            plt.close()
            self.last_save = time.time()
            self.get_logger().info(f'Gráfico actualizado :D: {SAVE_PATH}')

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
