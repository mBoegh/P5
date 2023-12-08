import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import threading
import serial

class SerialPublisherNode(Node):
    def __init__(self):
        super().__init__('serial_publisher_node')
        self.serial_port = '/dev/ttyACM0'  # Change this to your Arduino USB port
        self.serial_baudrate = 19200  # Change this to match your Arduino baudrate

        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=0  # Set timeout to 0 for non-blocking read
            )
            self.get_logger().info(f"Connected to {self.serial_port} at {self.serial_baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error connecting to {self.serial_port}: {e}")
            self.destroy_node()
            return

        self.publisher_ = self.create_publisher(Int16, 'serial_data', 10)
        self.serial_buffer = ""
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.start()

    def read_serial_data(self):
        while rclpy.ok():
            try:
                byte = self.serial_connection.read(1)
                try:
                    char = byte.decode('utf-8')
                    if char == '\n':
                        try:
                            data_value = int(self.serial_buffer)
                            msg = Int16()
                            msg.data = data_value
                            self.publisher_.publish(msg)
                            self.serial_buffer = ""
                            self.serial_connection.flush()  # Clear the input buffer
                        except ValueError:
                            self.get_logger().error(f"Received non-integer data: {self.serial_buffer}")
                    else:
                        self.serial_buffer += char
                except UnicodeDecodeError as e:
                    self.get_logger().error(f"Error decoding byte to utf-8: {e}")
            except serial.SerialException as e:
                self.get_logger().error(f"Error reading serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    serial_publisher_node = SerialPublisherNode()
    try:
        rclpy.spin(serial_publisher_node)
    except KeyboardInterrupt:
        pass
    serial_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
