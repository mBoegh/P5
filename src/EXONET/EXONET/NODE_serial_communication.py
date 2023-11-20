"""
TO DO:
 - Test live with Emotiv Epoc X
"""

from EXONET.EXOLIB import JSON_Handler, serial2arduino

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64


class Serial_Communication(Node, serial2arduino):
    """
    This is the Serial_Communication node of the EXONET ROS2 network.
    Takes argument(s):
     - serial_port (EG. COM3)
     - baud_rate (default 9600)
     - timeout (time (seconds) before connection attempt is aborted)
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, serial_port, baud_rate, bytesize, parity, stopbits, log_debug):

        # Initialising variables
        self.SERIAL_PORT = serial_port
        self.BAUD_RATE = baud_rate
        self.BYTESIZE = bytesize
        self.PARITY = parity
        self.STOPBITS = stopbits
        self.LOG_DEBUG = log_debug


        # Initialising the classes, from which this class is inheriting.
        Node.__init__(self, 'serial_communication')
        serial2arduino.__init__(self, self.SERIAL_PORT, self.BAUD_RATE, self.BYTESIZE, self.PARITY, self.STOPBITS, self.LOG_DEBUG)

        # Initialising a subscriber to the topic 'Motor_signals'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.motor_signals_subscription = self.create_subscription(Int64, 'Motor_signals', self.motor_signals_topic_callback, 10)
        self.motor_signals_subscription  # prevent unused variable warning

        # Initialising a publisher to the topic 'Feedback'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_publisher = self.create_publisher(Int64, 'Feedback', 10)
        self.feedback_publisher  # prevent unused variable warning

        self.get_logger().debug("Hello world!")

        # Establish a connection with the arduino
        self.arduino = self.establish_connection()

    def motor_signals_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'motor_signals_subscription'
        """

        # Log info
        self.get_logger().info(f"@ Class 'Serial_Communication' Function 'motor_signals_subscription'; Recieved data: '{msg.data}'")

        # Sending data to Arduino
        self.send_data(self.arduino, msg.data)

        # Initialize a variable of datatype std_msgs.msg.Int64 imported as Int64
        feedback_msg = Int64

        # Load feedback_msg with returned data 
        feedback_msg.data = self.receive_data(self.arduino)

        # Log info
        self.get_logger().info(f"@ Class 'Serial_Communication' Function 'motor_signals_subscription'; Received data: '{feedback_msg}'")

        # Publish signal with 'motor_signals_publisher' to topic 'Motor_signals'
        self.feedback_publisher.publish(feedback_msg)


####################
######  MAIN  ######
####################


def main():
    
    # Path for 'settings.json' file
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    SERIAL_PORT = handler.get_subkey_value("serial_communication", "SERIAL_PORT")
    BAUD_RATE = handler.get_subkey_value("serial_communication", "BAUD_RATE")
    BYTESIZE = handler.get_subkey_value("serial_communication", "BYTESIZE")
    PARITY = handler.get_subkey_value("serial_communication", "PARITY")
    STOPBITS = handler.get_subkey_value("serial_communication", "STOPBITS")
    LOG_DEBUG = handler.get_subkey_value("serial_communication", "LOG_DEBUG")

    # Initialize the rclpy library
    rclpy.init()

    # Instance the serverTCP class
    serial_communication = Serial_Communication(SERIAL_PORT, BAUD_RATE, BYTESIZE, PARITY, STOPBITS, LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(serial_communication)
    

if __name__ == "__main__":
    main()
    
