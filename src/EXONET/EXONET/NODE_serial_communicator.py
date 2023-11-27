"""
TO DO:
 - Test live with Emotiv Epoc X
"""

from EXONET.EXOLIB import JSON_Handler, serial2arduino

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int64, Float32

import time

class Serial_Communicator(Node, serial2arduino):
    """
    This is the Serial_Communicator node of the EXONET ROS2 network.
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

        # Flag for controlling what computations are done with the feedback signal.
        # If high then we get a time0 value
        # If low then we compute
        self.first_feedback = True
        self.time0 = None
        self.data0 = None

        # Initialize feedback message objects of datatype std_msgs.msg.Float32 imported as Float32
        self.feedback_joint_velocity_msg = Float32()
        self.feedback_joint_angle_msg = Float32()


        # Initialising the classes, from which this class is inheriting.
        Node.__init__(self, 'serial_communicator')
        serial2arduino.__init__(self, self.SERIAL_PORT, self.BAUD_RATE, self.BYTESIZE, self.PARITY, self.STOPBITS, self.LOG_DEBUG)

        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

        # Initialising a subscriber to the topic 'Motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int64 which is imported as Int64.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.velocity_motor_signals_subscription = self.create_subscription(Int64, 'Motor_signals', self.motor_signals_topic_callback, 10)
        self.velocity_motor_signals_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Manual_position_control_data'.
        # On this topic is expected data of type std_msgs.msg.UInt16 which is imported as UInt16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_position_control_data_subscriber = self.create_subscription(UInt16, 'Manual_position_control_data', self.manual_position_control_data_callback, 10)
        self.manual_position_control_data_subscriber

        # Initialising a publisher to the topic 'Feedback_joint_velocity'.
        # On this topic is expected data of type std_msgs.msg.FLoat32 which is imported as FLoat32.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_joint_velocity_publisher = self.create_publisher(Float32, 'Feedback_joint_velocity', 10)
        self.feedback_joint_velocity_publisher  # prevent unused variable warning

        # Initialising a publisher to the topic 'Feedback_joint_angle'.
        # On this topic is expected data of type std_msgs.msg.FLoat32 which is imported as FLoat32.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_joint_angle_publisher = self.create_publisher(Float32, 'Feedback_joint_angle', 10)
        self.feedback_joint_angle_publisher  # prevent unused variable warning

        # Establish a connection with the arduino
        self.arduino = self.establish_connection()

    def motor_signals_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'motor_signals_subscription'
        """

        # Log info
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Sending data to Arduino
        self.send_data(self.arduino, msg.data, seperator=",")


        time.sleep(2)

        self.get_logger().debug("post time.sleep")

        # Load feedback_msg with returned data 
        data = int(self.receive_data(self.arduino))


        self.get_logger().debug(f"Received serial data: '{data}'")


        if self.first_feedback:
            self.time0 = time.time()
            self.data0 = data

            self.first_feedback = False

        else:
            time_now = time.time()
        
            time_diff = time_now - self.time0

            self.time0 = time_now
        
            data_diff = data - self.data0

            self.data0 = data        
            
            j_vel = data_diff / time_diff  # Joint velocity

            elbow_joint_angle = self.map_range(1023-data, 0, 1023, -30, 210) # Joint angle 

            self.feedback_joint_velocity_msg.data = float(j_vel)
            self.feedback_joint_angle_msg.data = float(elbow_joint_angle)

            self.get_logger().debug(f"Computed 'j_vel' feedback data: '{j_vel}'")
            self.get_logger().debug(f"Computed 'elbow_joint_angle' feedback data: '{elbow_joint_angle}'")

            self.feedback_joint_angle_publisher.publish(self.feedback_joint_angle_msg)
            self.feedback_joint_velocity_publisher.publish(self.feedback_joint_velocity_msg)
            

    def manual_position_control_data_callback(self, msg):

        # Log info
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Sending data to Arduino
        self.send_data(self.arduino, msg.data, seperator= ",", state= "/")

        time.sleep(2)

        # Load feedback_msg with returned data 
        data = int(self.receive_data(self.arduino))

        self.get_logger().debug(f"Received serial data: '{data}'")


    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

####################
######  MAIN  ######
####################


def main():
    
    # Path for 'settings.json' file
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    SERIAL_PORT = handler.get_subkey_value("serial_communicator", "SERIAL_PORT")
    BAUD_RATE = handler.get_subkey_value("serial_communicator", "BAUD_RATE")
    BYTESIZE = handler.get_subkey_value("serial_communicator", "BYTESIZE")
    PARITY = handler.get_subkey_value("serial_communicator", "PARITY")
    STOPBITS = handler.get_subkey_value("serial_communicator", "STOPBITS")
    LOG_DEBUG = handler.get_subkey_value("serial_communicator", "LOG_DEBUG")
    LOG_LEVEL = handler.get_subkey_value("serial_communicator", "LOG_LEVEL")


    # Initialize the rclpy library
    rclpy.init()

    rclpy.logging.set_logger_level("serial_communicator", eval(LOG_LEVEL))

    # Instance the serverTCP class
    serial_communicator = Serial_Communicator(SERIAL_PORT, BAUD_RATE, BYTESIZE, PARITY, STOPBITS, LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(serial_communicator)
    

if __name__ == "__main__":
    main()
    
