"""
TO DO:
 - Test live with Emotiv Epoc X
"""

from EXONET.EXOLIB import JSON_Handler, serial2arduino, RunningAverage

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int64, Float32

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter

import time

class Serial_Communicator(Node, serial2arduino, RunningAverage):
    """
    This is the Serial_Communicator node of the EXONET ROS2 network.
    Takes argument(s):
     - serial_port (EG. COM3)
     - baud_rate (default 9600)
     - timeout (time (seconds) before connection attempt is aborted)
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, serial_port, baud_rate, bytesize, parity, stopbits, delay_between_sending_and_receiving, running_average_buffer_size, log_debug):

        # Initialising variables
        self.SERIAL_PORT = serial_port
        self.BAUD_RATE = baud_rate
        self.BYTESIZE = bytesize
        self.PARITY = parity
        self.STOPBITS = stopbits
        self.DELAY_BETWEEN_SENDING_AND_RECEIVING = delay_between_sending_and_receiving
        self.RUNNING_AVERAGE_BUFFER_SIZE = running_average_buffer_size
        self.LOG_DEBUG = log_debug

        # Flag for controlling what computations are done with the feedback signal.
        # If high then we get a time0 value
        # If low then we compute
        self.first_feedback = True
        self.time0 = None
        self.elbow_joint_angle_zero = None
        self.previous_velocity = 0

        # plots
        self.plot_data = []
        self.plot_elbow_joint_angle = []
        self.plot_j_vel = []
        self.plot_mean_j_vel = []


        # Initialize feedback message objects of datatype std_msgs.msg.Float32 imported as Float32
        self.feedback_joint_velocity_msg = Float32()
        self.feedback_joint_angle_msg = Float32()


        # Initialising the classes, from which this class is inheriting.
        Node.__init__(self, 'serial_communicator')
        serial2arduino.__init__(self, self.SERIAL_PORT, self.BAUD_RATE, self.BYTESIZE, self.PARITY, self.STOPBITS, self.LOG_DEBUG)
        RunningAverage.__init__(self, self.RUNNING_AVERAGE_BUFFER_SIZE)

        running_average_init_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        for data_point in running_average_init_values:
            self.add_data_point(data_point)

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
        self.send_data(self.arduino, msg.data, seperator="\n")

        time.sleep(self.DELAY_BETWEEN_SENDING_AND_RECEIVING)

        # Load feedback_msg with returned data 
        data = int(self.receive_data(self.arduino))


        self.get_logger().debug(f"Received serial data: '{data}'")


        if self.first_feedback:
            self.time0 = time.time()
            self.elbow_joint_angle_zero = self.map_range(1023-data, 0, 1023, -30, 210) # Joint angle 

            self.first_feedback = False

        else:

            time_now = time.time()

            elbow_joint_angle_now = self.map_range(1023-data, 0, 1023, -30, 210) # Joint angle 
        
            time_diff = time_now - self.time0

            self.time0 = time_now
        
            elbow_joint_angle_diff = elbow_joint_angle_now - self.elbow_joint_angle_zero

            self.elbow_joint_angle_zero = elbow_joint_angle_now

            self.data0 = data        
            
            j_vel = elbow_joint_angle_diff / time_diff  # Joint velocity

            # Compute running average using the RunningAverage object of the EXOLIB library with buffersize n defined in settings.json
            self.add_data_point(j_vel)
            mean_j_vel = self.get_average()

            self.feedback_joint_velocity_msg.data = float(mean_j_vel)
            self.feedback_joint_angle_msg.data = float(elbow_joint_angle_now)

            self.get_logger().debug(f"Computed 'j_vel' feedback data: '{mean_j_vel}'")
            self.get_logger().debug(f"Computed 'elbow_joint_angle' feedback data: '{elbow_joint_angle_now}'")

            if not self.previous_velocity == 0 and mean_j_vel > 10:
                self.feedback_joint_angle_publisher.publish(self.feedback_joint_angle_msg)
                self.feedback_joint_velocity_publisher.publish(self.feedback_joint_velocity_msg)
            else:
                self.feedback_joint_velocity_msg.data = float(0)

                self.feedback_joint_angle_publisher.publish(self.feedback_joint_angle_msg)
                self.feedback_joint_velocity_publisher.publish(self.feedback_joint_velocity_msg)

            self.previous_velocity = mean_j_vel

            self.plot_data.append(data)
            self.plot_elbow_joint_angle.append(elbow_joint_angle_now)
            self.plot_j_vel.append(j_vel)
            self.plot_mean_j_vel.append(mean_j_vel)

    def manual_position_control_data_callback(self, msg):

        # Log info
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Sending data to Arduino
        self.send_data(self.arduino, msg.data, seperator= "\n", state= "/")

        time.sleep(self.DELAY_BETWEEN_SENDING_AND_RECEIVING)

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
    DELAY_BETWEEN_SENDING_AND_RECEIVING = handler.get_subkey_value("serial_communicator", "DELAY_BETWEEN_SENDING_AND_RECEIVING")
    RUNNING_AVERAGE_BUFFER_SIZE = handler.get_subkey_value("serial_communicator", "RUNNING_AVERAGE_BUFFER_SIZE")
    LOG_DEBUG = handler.get_subkey_value("serial_communicator", "LOG_DEBUG")
    LOG_LEVEL = handler.get_subkey_value("serial_communicator", "LOG_LEVEL")


    # Initialize the rclpy library
    rclpy.init()

    rclpy.logging.set_logger_level("serial_communicator", eval(LOG_LEVEL))

    # Instance the serverTCP class
    serial_communicator = Serial_Communicator(SERIAL_PORT, BAUD_RATE, BYTESIZE, PARITY, STOPBITS, DELAY_BETWEEN_SENDING_AND_RECEIVING, RUNNING_AVERAGE_BUFFER_SIZE, LOG_DEBUG)

    iter = 0
    while iter != 1000:
        # Begin looping the node
        rclpy.spin_once(serial_communicator)
        iter += 1

    plt.plot(serial_communicator.plot_data)
    plt.ylabel('Potentiometer data')
    plt.ylim((0,1023))
    plt.show()

    plt.plot(serial_communicator.plot_elbow_joint_angle)
    plt.ylabel('Elbow joint angle')
    plt.ylim((40,170))
    plt.show()

    plt.plot(serial_communicator.plot_j_vel)
    plt.ylabel('Computed joint velocity')
    plt.show()


    plt.plot(serial_communicator.plot_mean_j_vel)
    plt.ylabel('Computed runnning average joint velocity')
    plt.show()

if __name__ == "__main__":
    main()
    
