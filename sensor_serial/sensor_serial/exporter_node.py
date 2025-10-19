import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from prometheus_client import Gauge, start_http_server

TEMP_GAUGE = Gauge('temperature_celsius',
                   'Processed temperature from ROS (sensor_serial)',
                   ['sensor_id'])

class ExporterNode(Node):
    def __init__(self):
        super().__init__('exporter_node')
        self.sensor_id = 'arduino_1'

        # QoS configurable: best_effort (default seguro) o reliable
        qos_mode = self.declare_parameter('qos_mode', 'best_effort') \
                        .get_parameter_value().string_value.lower()
        if qos_mode == 'reliable':
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        self.sample = TEMP_GAUGE.labels(sensor_id=self.sensor_id)
        self.sample.set(float('nan'))

        self.sub = self.create_subscription(
            Float32,
            'temperature_celsius',
            self._on_msg,
            qos_profile
        )

        self.get_logger().info(f"ExporterNode listo en :8000/metrics (QoS: {qos_mode.upper()})")

    def _on_msg(self, msg: Float32):
        val = float(msg.data)
        self.sample.set(val)
        self.get_logger().info(f"Exporter recibió: {val:.2f} °C")

def main(args=None):
    rclpy.init(args=args)
    start_http_server(8000)
    node = ExporterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

