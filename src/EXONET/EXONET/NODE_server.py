"""
TO DO:
 - Test live with Emotiv Epoc X.
"""

from EXONET.EXOLIB import JSON_Handler
from EXONET.EXOLIB import TCP_Server
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Server(Node, TCP_Server):
    """
    This is the server node of the EXONET ROS2 network.
    Takes argument(s):
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, host, port, timer_period, log_debug):

        # Initialising variables
        self.HOST = host
        self.PORT = port
        self.TIMER_PERIOD = timer_period
        self.LOG_DEBUG = log_debug


        # Initialising the classes, from which this class is inheriting.
        Node.__init__(self, 'server')
        TCP_Server.__init__(self, self.HOST, self.PORT, self.LOG_DEBUG)

        # Waits for incomming connection to TCP server
        self.connection = self.await_connection()

        # Create a timer which periodically calls the specified callback function at a defined interval.
        # Initialise timer_counter as zero. This is iterated on each node spin
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)
        self.timer_counter = 0

        # Initialising a publisher to the topic 'EEG_data'.
        # On this topic is published data of type std_msgs.msg.String which is imported as String.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_publisher = self.create_publisher(String, 'EEG_data', 10)
        self.eeg_data_publisher  # prevent unused variable warning

    def timer_callback(self):
        
        # Initialise variable msg as being of data type 'std_msgs.msg.String' imported as String
        msg = String()

        # Load msg with EEG data recieved on the TCP server 
        msg.data = self.recieve_data_loop()

        # Publish msg using eeg_data_publisher on topic 'EEG_data'
        self.eeg_data_publisher.publish(msg)

        if self.LOG_DEBUG:
            self.get_logger().info(f"@ Class 'Server' Function 'eeg_data_topic_callback'; Published data: '{msg.data}'")

        # Iterate timer
        self.timer_counter += 1


####################
######  MAIN  ######
####################

def main():

    # Path for 'settings.json' file
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    handler = JSON_Handler(json_file_path)

    # Get settings from 'settings.json' file and save them to their respective variables
    HOST = handler.get_subkey_value("server", "HOST")
    PORT = handler.get_subkey_value("server", "PORT")
    TIMER_PERIOD = handler.get_subkey_value("server", "TIMER_PERIOD")
    LOG_DEBUG = handler.get_subkey_value("server", "LOG_DEBUG")

    # Initialize the rclpy library
    rclpy.init()

    # Instance the serverTCP class
    server = Server(HOST, PORT, TIMER_PERIOD, LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(server)

if __name__ == "__main__":
    main()
