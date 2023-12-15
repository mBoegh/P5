from EXONET.EXOLIB import JSON_Handler, TCP_Server        

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class Server(Node, TCP_Server):
    """
    This is the server node of the EXONET ROS2 network.
    The purpose of the Server is to recieve and distribute EEG data stream from Node-Red using TCP/IP.
    """

    def __init__(self, host, port, timer_period, log_debug):

        # Initialising parsed Variables.
        self.HOST = host
        self.PORT = port
        self.TIMER_PERIOD = timer_period
        self.LOG_DEBUG = log_debug

        # Initialising class Variables.
        self.init_callback = True
        self.toggle_EEG_parameter = False

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'server')

        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

        # Initialising variable msg as being of data type 'std_msgs.msg.String' imported as String.
        # The message is loaded with data and published to a topic.
        self.msg = String()

        # Create a timer which periodically calls the specified callback function at a defined interval.
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

        # Initialising a publisher to the topic 'EEG_data'.
        # On this topic is published data of type std_msgs.msg.String which is imported as String.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_publisher = self.create_publisher(String, 'EEG_data', 10)
        self.eeg_data_publisher  # prevent unused variable warning

        # Initialising a publisher to the topic 'EEG_toggle'.
        # On this topic is published data of type std_msgs.msg.Bool which is imported as Bool.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_toggle_subscribtion = self.create_subscription(Bool, 'EEG_toggle', self.eeg_toggle_topic_callback, 10)
        self.eeg_toggle_subscribtion  # prevent unused variable warning
    

    def eeg_toggle_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'eeg_toggle_subscribtion'.
        """
        
        # Log received message data, as debug level of importance.
        self.get_logger().debug(f"Received data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is as a boolean value.
        value = msg.data

        # This statement is true if the received boolean value is True 
        # and if it is the first time a message is received, which is handled by the init_callback flag.
        # Handles the init establishing of connection using TCP/IP to any connecting client.
        if value and self.init_callback:

            # try / except statement used for catching errors, logging them and continuing with program execution.
            try:

                # Log that the system is waiting for any incoming TCP/IP connections on the HOST IP and PORT, which is set in settings.json, as info level of importance.
                self.get_logger().info(f"Waiting for connection at 'IP:PORT': '{self.HOST}:{self.PORT}'.")

                # Instance the TCP_Server class to create a socket connection using parsed arguments.
                TCP_Server.__init__(self, self.HOST, self.PORT, self.LOG_DEBUG)
                
                # Waits for any incomming connection to the TCP server socket.
                self.connection = self.await_connection()

                # If a connection is successfully established, then the init_flag is lowered.
                # This makes it so that the system wont attempt to connect whilst already connected, 
                # while still letting the user determine whether the system shall react to the 
                # EEG data stream or not using the toggle switch in the GUI node.
                self.init_callback = False

                # Log that the system has successfully established a connection with a client, as info level of importance.
                self.get_logger().info(f"Successfully connected.")

                # Set the toggle_EEG_parameter true, thereby letting the system know that it shall react to the EEG data stream.
                self.toggle_EEG_parameter = True
                            
                # Log that the system recognised that the user toggled the EEG switch in the GUI node, as debug level of importance.
                self.get_logger().debug(f"Tooggled EEG True.")

            except Exception as e:

                # Log the exception formatted with the context of what the system attempted, as error level of importance.
                self.get_logger().error(f"Failed connecting with error: {e}")

        # This statement is true if the received boolean value is True.
        # Toggles the toggle_EEG_parameter true thereby letting the system know that it shall react to the EEG data stream.
        elif value:

            # Set the toggle_EEG_parameter true, thereby letting the system know that it shall react to the EEG data stream.
            self.toggle_EEG_parameter = True

            # Log that the system recognised that the user toggled the EEG switch in the GUI node to true, as debug level of importance.
            self.get_logger().debug(f"Toggled EEG True.")
        
        # This statement is true if the received boolean value is False.
        # Toggles the toggle_EEG_parameter false thereby letting the system know that it shall not react to the EEG data stream.
        elif not value:

            # Set the toggle_EEG_parameter false, thereby letting the system know that it shall not react to the EEG data stream.
            self.toggle_EEG_parameter = False

            # Log that the system recognised that the user toggled the EEG switch in the GUI node to false, as debug level of importance.
            self.get_logger().debug(f"Toggled EEG False.")
        
        else:

            # Log that none of the above statements were true, meaning that the received message was unconventional, as warning level of importance.
            self.get_logger().warning(f"Unexpected message data on topic.")


    def timer_callback(self):
        """
        Function called at specific time interval, specified in 'settings.json'.
        """
        
        # This statement is true if the toggle_EEG_parameter is true. This is toggled by pressing the EEG toggle switch in the GUI node.
        # If true then the msg string message is loaded with data received from the client on TCP/IP, and published to the 'EEG_data' topic.
        if self.toggle_EEG_parameter:

            # Load msg with EEG data recieved from the TCP client.
            self.msg.data = self.recieve_data_loop(self.connection)

            # Log received data, as debug level of importance.
            self.get_logger().debug(f"Recieved data from client: '{self.msg.data}'")

            # This statement is true if the data recieved from the TCP client is not empty. If the data is empty, then the connection has been broken. Therefore no more data will arrive and the system shall attempt to reconnect.
            # If true then the msg string message is published to the 'EEG_data' topic.
            if not self.msg.data == "":                

                # Publish msg using eeg_data_publisher on topic 'EEG_data'.
                self.eeg_data_publisher.publish(self.msg)

                # Log published data, as debug level of importance.
                self.get_logger().debug(f"Published data: '{self.msg.data}'")

            else:

                # Log that the data recieved from the client is empty, as debug level of importance.
                self.get_logger().warning(f"Recieved nothing from client.")

                # Await a new client connection.
                self.connection = self.await_connection()
        

####################
######  MAIN  ######
####################

def main():
    """
    Main function of the Server node in the EXONET ROS2 Humble network.
    """

    # Path for 'settings.json' file.
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file.
    handler = JSON_Handler(json_file_path)

    # Get settings from 'settings.json' file.
    HOST = handler.get_subkey_value("server", "HOST")
    PORT = handler.get_subkey_value("server", "PORT")
    TIMER_PERIOD = handler.get_subkey_value("server", "TIMER_PERIOD")
    LOG_DEBUG = handler.get_subkey_value("server", "LOG_DEBUG")
    LOG_LEVEL = handler.get_subkey_value("server", "LOG_LEVEL")

    # Initialize the rclpy library.
    rclpy.init()

    # Sets the logging level of importance. 
    # When setting, one is setting the lowest level of importance one is interested in logging.
    # Logging level is defined in settings.json.
    # Logging levels:
    # - DEBUG
    # - INFO
    # - WARNING
    # - ERROR
    # - FATAL
    # The eval method interprets a string as a command.
    rclpy.logging.set_logger_level("server", eval(LOG_LEVEL))

    # Instance the node class.
    server = Server(HOST, PORT, TIMER_PERIOD, LOG_DEBUG)

    # Begin looping the node.
    rclpy.spin(server)

if __name__ == "__main__":
    main()
