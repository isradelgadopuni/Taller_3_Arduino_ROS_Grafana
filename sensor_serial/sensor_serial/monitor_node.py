import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32


class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # Parámetro de QoS: puedes pasar --ros-args -p qos_mode:=best_effort o reliable
        self.qos_mode = self.declare_parameter('qos_mode', 'reliable') \
                             .get_parameter_value().string_value.lower()

        # Define el perfil QoS según el parámetro
        if self.qos_mode == 'best_effort':
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else:  # reliable por defecto
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        self.get_logger().info(f"MonitorNode iniciado con QoS: {self.qos_mode.upper()}")

        # Suscripción al tópico de temperatura
        self.subscription = self.create_subscription(
            Float32,
            'temperature_celsius',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg: Float32):
        self.get_logger().info(f'Temperatura actual: {msg.data:.2f} °C')


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

