"""
TO DO:
 - Make UI with visualisation of detected mental command and its power (Eg. 'Lift' or 'Drop' and power 0-100)
"""

from EXONET.EXOLIB import JSON_Handler
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64


class Visualizer(Node):
    """
    This is the visualizer node of the EXONET ROS2 network.
    Takes argument(s):
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, log_debug):

        # Initialising variables
        self.LOG_DEBUG = log_debug

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('visualizer')

        # Initialising a subscriber to the topic 'EEG_data'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_subscription = self.create_subscription(String, 'EEG_data', self.eeg_data_topic_callback, 10)
        self.eeg_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'EEG_data'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.motor_signals_subscription = self.create_subscription(String, 'Motor_signals', self.motor_signals_topic_callback, 10)
        self.motor_signals_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Feedback'.
        # On this topic is expected data of type std_msgs.msg.Int64 which is imported as Int64.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_subscription = self.create_subscription(Int64, 'Feedback', self.feedback_topic_callback, 10)
        self.feedback_subscription  # prevent unused variable warning

        """
        self.get_logger().debug("DEBUG Hello world!")
        self.get_logger().info("INFO Hello world!")
        self.get_logger().warning("WARNING Hello world!")
        self.get_logger().error("ERROR Hello world!")
        """

    def eeg_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'eeg_data_subscription'
        """

        # Log info
        self.get_logger().info(f"@ Class 'Visualizer' Function 'eeg_data_topic_callback'; Received data: '{msg.data}'")


    def motor_signals_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'motor_signals_subscription'
        """

        # Log info
        self.get_logger().info(f"@ Class 'Visualizer' Function 'motor_signals_topic_callback'; Recieved data: '{msg.data}'")


    def feedback_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'feedback_subscription'
        """

        # Log info
        self.get_logger().info(f"@ Class 'Controller' Function 'feedback_topic_callback'; Recieved data '{msg.data}'")


####################
######  MAIN  ######
####################


def main():
    
    # Path for 'settings.json' file
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    LOG_DEBUG = handler.get_subkey_value("visualizer", "LOG_DEBUG")

    # Initialize the rclpy library
    rclpy.init()

    # Instance the serverTCP class
    visualizer = Visualizer(LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(visualizer)
    

if __name__ == "__main__":
    main()
    