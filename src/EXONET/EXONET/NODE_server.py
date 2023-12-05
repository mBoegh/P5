from EXONET.EXOLIB import JSON_Handler, TCP_Server        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


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

        self.init_callback = True
        self.toggle_EEG_parameter = False

        # Initialising the classes, from which this class is inheriting.
        Node.__init__(self, 'server')

        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

        # Create a timer which periodically calls the specified callback function at a defined interval.
        # Initialise timer_counter as zero. This is iterated on each node spin
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)
        self.timer_counter = 0

        # Initialising a publisher to the topic 'EEG_data'.
        # On this topic is published data of type std_msgs.msg.String which is imported as String.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_publisher = self.create_publisher(String, 'EEG_data', 10)
        self.eeg_data_publisher  # prevent unused variable warning


        self.eeg_toggle_subscriber = self.create_subscription(Bool, 'EEG_toggle', self.eeg_toggle_topic_callback, 10)
        self.eeg_toggle_subscriber  # prevent unused variable warning
    

    def eeg_toggle_topic_callback(self, msg):
        
        value = msg.data

        if value and self.init_callback:
            
            self.get_logger().debug(f"Tooggled EEG True")

            try:
                self.get_logger().info(f"Attempting connection")

                # Instance the TCP_Server class to create a socket connection using defined parameters
                TCP_Server.__init__(self, self.HOST, self.PORT, self.LOG_DEBUG)
                
                # Waits for incomming connection to TCP server
                self.connection = self.await_connection()

                self.init_callback = False

                self.get_logger().info(f"Successfully connected")

            except Exception as e:
                self.get_logger().error(f"Failed connecting with error: {e}")


        elif value:
            self.get_logger().debug(f"Toggled EEG True")

            self.toggle_EEG_parameter = True
        
        elif not value:
            self.get_logger().debug(f"Toggled EEG False")

            self.toggle_EEG_parameter = False
        
        else:
            self.get_logger().warning(f"Unexpected message data on topic.")


    def timer_callback(self):
        
        if self.toggle_EEG_parameter:
            # Initialise variable msg as being of data type 'std_msgs.msg.String' imported as String
            msg = String()

            # Load msg with EEG data recieved on the TCP server 
            msg.data = self.recieve_data_loop()

            # Publish msg using eeg_data_publisher on topic 'EEG_data'
            self.eeg_data_publisher.publish(msg)

            # Log info
            self.get_logger().debug(f"Published data: '{msg.data}'")

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
    LOG_LEVEL = handler.get_subkey_value("server", "LOG_LEVEL")


    # Initialize the rclpy library
    rclpy.init()

    rclpy.logging.set_logger_level("server", eval(LOG_LEVEL))

    # Instance the serverTCP class
    server = Server(HOST, PORT, TIMER_PERIOD, LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(server)

if __name__ == "__main__":
    main()
