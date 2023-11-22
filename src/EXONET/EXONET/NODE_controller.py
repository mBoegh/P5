"""
TO DO:
 - Create controll system
"""

from EXONET.EXOLIB import JSON_Handler
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Bool

from simple_pid import PID

class Controller(Node):


    """
    This is the Controller node of the EXONET ROS2 network.
    Takes argument(s):
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, log_debug):

        # D should always be 0, Don't change setpoint!!! 
        self.pi = PID(1, 0, 0, setpoint=1)

        print("Hello World!")

        # Initialising variables
        self.LOG_DEBUG = log_debug
        self.toggle_EEG_parameter = True


        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('controller')


        # Initialising a subscriber to the topic 'EEG_toggle'.
        # On this topic is expected data of type std_msgs.msg.Bool which is imported as Bool.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_toggle_subscription = self.create_subscription(Bool, 'EEG_toggle', self.eeg_toggle_topic_callback, 10)
        self.eeg_toggle_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'EEG_data'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_subscription = self.create_subscription(String, 'EEG_data', self.eeg_data_topic_callback, 10)
        self.eeg_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Manual_control_data'.
        # On this topic is expected data of type std_msgs.msg.Int8 which is imported as Int8.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_control_data_subscription = self.create_subscription(Int8, 'Manual_control_data', self.manual_control_data_topic_callback, 10)
        self.manual_control_data_subscription  # prevent unused variable warning

        # Initialising a publisher to the topic 'Motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int8 which is imported as Int8.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.motor_signals_publisher = self.create_publisher(Int8, 'Motor_signals', 10)
        self.motor_signals_publisher  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Feedback'.
        # On this topic is expected data of type std_msgs.msg.Int8 which is imported as Int8.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_subscription = self.create_subscription(Int8, 'Feedback', self.feedback_topic_callback, 10)
        self.feedback_subscription  # prevent unused variable warning
    

    def eeg_toggle_topic_callback(self, msg):
        
        value = msg.data

        if value:
            self.get_logger().debug(f"@ Class 'Controller' Function 'callback_eeg_toggle'; Tooggled EEG True")

            self.toggle_EEG_parameter = True
        
        elif not value:
            self.get_logger().debug(f"@ Class 'Controller' Function 'callback_eeg_toggle'; Tooggled EEG False")

            self.toggle_EEG_parameter = False
        
        else:
            self.get_logger().warning(f"@ Class 'Controller' Function 'callback_eeg_toggle'; Unexpected message data on topic.")


    def eeg_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'manual_data_subscription'
        """

        if self.toggle_EEG_parameter:

            # Log info
            self.get_logger().debug(f"@ Class 'Controller' Function 'eeg_data_topic_callback'; Recieved data '{msg.data}'")


            t_vel = 70 # Target velocity
            j_vel = 100 # Joint velocity
            volt = 10   # Current voltage

            error = 1 - (j_vel/t_vel)
            control = self.pi(error)
            volt_p = volt/control


            self.signal = volt_p

            # redifine msg to be of datatype std_msgs.msg.Int8 which is imported as Int8
            msg = Int8

            # Load msg with signal data 
            msg.data = self.signal

            # Log info
            self.get_logger().debug(f"@ Class 'Controller' Function 'eeg_data_topic_callback'; Publishing data '{msg.data}'")

            # Publish signal with 'motor_signals_publisher' to topic 'Motor_signals'
            self.motor_signals_publisher.publish(msg)

    
    def manual_control_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'manual_control_subscription'
        """

        if not self.toggle_EEG_parameter:

            # Log info
            self.get_logger().debug(f"@ Class 'Controller' Function 'eeg_data_topic_callback'; Recieved data '{msg.data}'")


            t_vel = 70 # Target velocity
            j_vel = 100 # Joint velocity
            volt = 10   # Current voltage

            error = 1 - (j_vel/t_vel)
            control = self.pi(error)
            volt_p = volt/control


            self.signal = volt_p

            # redifine msg to be of datatype std_msgs.msg.Int8 which is imported as Int8
            msg = Int8

            # Load msg with signal data 
            msg.data = self.signal

            # Log info
            self.get_logger().debug(f"@ Class 'Controller' Function 'eeg_data_topic_callback'; Publishing data '{msg.data}'")

            # Publish signal with 'motor_signals_publisher' to topic 'Motor_signals'
            self.motor_signals_publisher.publish(msg)


    def feedback_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'feedback_subscription'
        """

        # Log info
        self.get_logger().debug(f"@ Class 'Controller' Function 'feedback_topic_callback'; Recieved data '{msg.data}'")


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
    controller = Controller(LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(controller)
    

if __name__ == "__main__":
    main()
    