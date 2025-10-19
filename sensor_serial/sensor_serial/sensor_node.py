import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Parámetros (puedes cambiarlos desde CLI si lo necesitas)
        self.port = self.declare_parameter('port', '/dev/ttyACM0') \
                        .get_parameter_value().string_value
        self.baud = self.declare_parameter('baud', 9600) \
                        .get_parameter_value().integer_value

        # Abre el puerto serial (Arduino)
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f'Abierto {self.port} @ {self.baud} baudios')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir {self.port}: {e}')
            raise

        # Publisher al tópico sensor_data (Int32)
        self.publisher_ = self.create_publisher(Int32, 'sensor_data', 10)

        # Timer a 1 Hz
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.isdigit():  # el ADC del ejemplo siempre es no negativo
                value = int(line)
                msg = Int32()
                msg.data = value
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publicando crudo: {value}')
            else:
                if line:  # solo loguea si llegó algo no vacío
                    self.get_logger().warn(f'Dato inválido: "{line}"')
        except Exception as e:
            self.get_logger().error(f'Error leyendo serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

