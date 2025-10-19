import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Float32


class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')

        # --- Parámetro para el QoS ---
        # Puedes pasar desde CLI:  --ros-args -p qos_mode:=reliable  o  best_effort
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

        self.get_logger().info(f"ProcessorNode iniciado con QoS: {self.qos_mode.upper()}")

        # Suscripción al tópico crudo
        self.subscription = self.create_subscription(
            Int32,
            'sensor_data',
            self.listener_callback,
            10  # el subs interno puede quedar con QoS simple
        )

        # Publisher al tópico procesado (aplica QoS seleccionado)
        self.publisher_ = self.create_publisher(
            Float32,
            'temperature_celsius',
            qos_profile
        )

    def listener_callback(self, msg: Int32):
        raw_value = msg.data
        # Escala 0..1023 → 0..100 °C
        temperature = (raw_value * 1.000)

        temp_msg = Float32()
        temp_msg.data = temperature
        self.publisher_.publish(temp_msg)
        self.get_logger().info(f'Procesado: {temperature:.2f} C')


def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

