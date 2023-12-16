from EXONET.EXOLIB import JSON_Handler, Serial_to_microcontroller, RunningAverage

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int64, Float32, Int16, String

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal

import time

class Serial_Communicator(Node, Serial_to_microcontroller):
    """
    This is the Serial_Communicator node of the EXONET ROS2 network.
    The purpose of the Serial_Communicator is establishing a serial communication
    with a micro controller and sending & receiving data.
    """

    def __init__(self, serial_port, baud_rate, bytesize, parity, stopbits, delay_between_sending_and_receiving, running_average_buffer_size, log_debug):

        # Initialising parsed Variables
        self.SERIAL_PORT = serial_port
        self.BAUD_RATE = baud_rate
        self.BYTESIZE = bytesize
        self.PARITY = parity
        self.STOPBITS = stopbits
        self.DELAY_BETWEEN_SENDING_AND_RECEIVING = delay_between_sending_and_receiving
        self.RUNNING_AVERAGE_BUFFER_SIZE = running_average_buffer_size
        self.LOG_DEBUG = log_debug


        # Initialising class Variables
        self.first_feedback = True  # Flag is lowered after first feedback message
        self.second_feedback = True  # Flag is lowered after second feedback message
        self.program_start_time = None  # Contains the time stamp of the program start.
        self.time_zero = None  # Acts as a previous time stamp. Used in computing time difference since last received serial message.
        self.elbow_joint_angle_zero = None  # Acts as a previous data point. Used in computing velocity, by measuring angle difference over time difference.
        self.previous_velocity = 0  # Acts as a previous data point. Used in computing velocity, by measuring angle difference over time difference.
        self.t_vel = 0  # Contains the current target velocity. Only used in plotting.

        # Initialising the data lists for plotting using MatPlotLib
        self.plot_time = []  # Contains the time axis.
        self.plot_mean_elbow_joint_angle = []  # Contains the smoothed elbow joint angle.
        self.plot_j_vel = []  # Contains the smoothed joint velocity.
        self.plot_t_vel = []  # Contains the target velocity.

        # Initialising variable feedback_joint_velocity_msg and feedback_joint_angle_msg as being of data type 'std_msgs.msg.Float32' imported as Float32.
        # The messages is loaded with data and published to a topic.
        self.feedback_joint_velocity_msg = Float32()
        self.feedback_joint_angle_msg = Float32()

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        Node.__init__(self, 'serial_communicator')

        # Initialising the 'Serial_to_microcontroller' class, from which this class is inheriting.
        Serial_to_microcontroller.__init__(self, self.SERIAL_PORT, self.BAUD_RATE, self.BYTESIZE, self.PARITY, self.STOPBITS, self.LOG_DEBUG)
        
        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

        # Initialising a subscriber to the topic 'Velocity_motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.velocity_motor_signals_subscription = self.create_subscription(Int16, 'Velocity_motor_signals', self.motor_signals_topic_callback, 10)
        self.velocity_motor_signals_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Manual_velocity_control_data'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_input_velocity_control_data_subscription = self.create_subscription(Int16, 'Manual_velocity_control_data', self.motor_signals_topic_callback, 10)
        self.manual_input_velocity_control_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Manual_position_control_data'.
        # On this topic is expected data of type std_msgs.msg.UInt16 which is imported as UInt16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_position_control_data_subscription = self.create_subscription(UInt16, 'Manual_position_control_data', self.manual_position_control_data_callback, 10)
        self.manual_position_control_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Target_velocity'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.target_velocity_topic_subscription = self.create_subscription(Int16, 'Target_velocity', self.target_velocity_topic_callback, 10)
        self.target_velocity_topic_subscription  # prevent unused variable warning

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

        # Establish a connection with the microcontroller
        self.microcontroller = self.establish_connection()


    def target_velocity_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Target_velocity' using the subscription 'target_velocity_topic_subscription'.
        """
        
       # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Unpack the recieved message data by reinitializing t_vel.
        # The expected data is formatted as a signed integer with limits set by the method of generating the target velocity.
        self.t_vel = msg.data


    def motor_signals_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on either the topic 'Velocity_motor_signals' using the subscription 'velocity_motor_signals_subscription'
        or the topic 'Manual_velocity_control_data' using the subscription 'manual_input_velocity_control_data_subscription'.
        """
        
        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is formatted as an integer from -100 to 100.
        duty_cycle = msg.data

        # Format the data as a message to be sent over serial to the microcontroller,
        # such that it follows the established encoding convention of the system.
        # The duty cycle is increased by 1000, which will be retracted again on the microcontroller.
        # This is to handle the value as uint in transit to the microcontroller, and int in use on the microcontroller.
        # Furthermore the message is formatted as a string with a zero padded at the front.
        # This is due to the current microcontroller, an Arduino Leonardo, removing the first scipher of every message,
        # when it recieves messages at a high frequency.
        serial_message = f"0{duty_cycle + 1000}"

        # Sending data to microcontroller.
        self.send_data(self.microcontroller, serial_message, seperator="\n")

        # This is an optional wait time between the system sends a message over serial
        # and recieves a message over serial.
        # The value DELAY_BETWEEN_SENDING_AND_RECEIVING is set in settings.json.
        time.sleep(self.DELAY_BETWEEN_SENDING_AND_RECEIVING)

        # Recieve serial data and cast it as integer.
        data = int(self.receive_data(self.microcontroller))

        # Log received serial data as debug level of importance.
        self.get_logger().debug(f"Received serial data: '{data}'")

        # This statement is true when the first_feedback flag is true, which it is at init.
        # If true then a time_zero time reading is stamped, and the program start time is stamped as time_zero.
        # The first current exoskeleton joint angle is measured, initialized as elbow_joint_angle_zero,
        # and the RunningAverage class is instanced with init values being the current exoskeleton elbow joint angle.
        # Then the first_feedback is lowered, so that this section does not run again.
        if self.first_feedback:

            # First time stamp.
            self.time_zero = time.time()

            # Saving the first time stamp.
            self.program_start_time = self.time_zero

            # Computing the current exoskeleton elbow joint angle by mapping the potentiometer 
            # reading recieved from the microcontroller to be in the range of the exoskeleton joint.
            # This computation acts as the previous reading when computing exoskeleton elbow joint velocity,
            # by comparing the difference in joint angle over the difference in time.
            self.elbow_joint_angle_zero = self.map_range(data, 0, 1023, -30, 210)  # Joint angle

            # Instancing the RunningAverage class. Buffer size is determined in settings.json as RUNNING_AVERAGE_BUFFER_SIZE.
            # The class is initialised by filling the entire buffer with the first exoskeleton elbow joint angle computation. 
            self.running_average = RunningAverage(RUNNING_AVERAGE_BUFFER_SIZE, self.elbow_joint_angle_zero)

            # Lower the first_feedback flag, so that this section does not run again.
            self.first_feedback = False
        
        else:

            # Time stamping the current time.
            time_now = time.time()

            # Computing the current exoskeleton elbow joint angle by mapping the potentiometer 
            # reading recieved from the microcontroller to be in the range of the exoskeleton joint.
            elbow_joint_angle_now = self.map_range(data, 0, 1023, -30, 210)
        
            # Adding new datapoint to the buffer of the running average for the exoskeleton elbow joint angle and removing the oldest data point.
            self.running_average.add_data_point(elbow_joint_angle_now)  # Joint angle

            # Computing the running average with the new buffer.
            mean_elbow_joint_angle = self.running_average.get_average()

            # Logs the computed exoskeleton mean elbow joint angle as debug level of importance.
            self.get_logger().debug(f"Computed 'mean_elbow_joint_angle' feedback data: '{mean_elbow_joint_angle}'")
        
            # Computing the time difference between the current time stamp and the previous time stamp.
            time_diff = time_now - self.time_zero

            # Resetting the previous time stamp to be the time of the current time stamp.
            self.time_zero = time_now
        
            # Computing the difference in exoskeleton elbow joint angle between the current angle and the previous angle.
            elbow_joint_angle_diff = mean_elbow_joint_angle - self.elbow_joint_angle_zero

            # Resetting the previous angle to be the angle of the current angle.
            self.elbow_joint_angle_zero = mean_elbow_joint_angle
            
            # Computing exoskeleton elbow joint velocity by dividing the difference in angle with the difference in time.
            j_vel = elbow_joint_angle_diff / time_diff  # Joint velocity

            # This statement is true when the second_feedback flag is true, which it is at init.
            # If true then instance the RunningAverage class with init values being the 
            # the current exoskeleton elbow joint velocity.
            # Then the mean exoskeleton elbow joint velocity is initialized as the value of the first joint velocity computation.
            # Then the second_feedback is lowered, so that this section does not run again.
            if self.second_feedback:
                
                # Instancing the RunningAverage class. Buffer size is determined in settings.json as RUNNING_AVERAGE_BUFFER_SIZE.
                # The class is initialised by filling the entire buffer with the first exoskeleton elbow joint velocity computation. 
                self.running_average_vel = RunningAverage(RUNNING_AVERAGE_BUFFER_SIZE, j_vel)
                
                # The mean exoskeleton elbow joint velocity is initialized as the value of the first joint velocity computation.
                mean_elbow_joint_vel = j_vel
            
                # Lower the second_feedback flag, so that this section does not run again.
                self.second_feedback = False

            else:

                # Adding new datapoint to the buffer of the running average for the exoskeleton elbow joint velocity and removing the oldest data point.
                self.running_average_vel.add_data_point(j_vel)
                
                # Computing the running average with the new buffer.
                mean_elbow_joint_vel = self.running_average_vel.get_average()

                # Logs the computed exoskeleton mean elbow joint velocity as debug level of importance.
                self.get_logger().debug(f"Computed 'mean_elbow_joint_vel' feedback data: '{mean_elbow_joint_vel}'")
        
            # Loads the feedback_joint_velocity_msg with the computed mean joint velocity.
            self.feedback_joint_velocity_msg.data = float(mean_elbow_joint_vel)

            # Loads the feedback_joint_angle_msg with the computed mean joint angle.
            self.feedback_joint_angle_msg.data = float(mean_elbow_joint_angle)

            # Publishes the computed feedback joint angle to the topic /Feedback_joint_angle.
            self.feedback_joint_angle_publisher.publish(self.feedback_joint_angle_msg)

            # Logs the published feedback_joint_angle_msg data with Debug level of importance.
            self.logger.debug(f"Published 'feedback_joint_angle_msg' data: '{self.feedback_joint_angle_msg.data}'")

            # Publishes the computed feedback joint velocity to the topic /Feedback_joint_velocity.
            self.feedback_joint_velocity_publisher.publish(self.feedback_joint_velocity_msg)

            # Logs the published feedback_joint_velocity_msg data with Debug level of importance.
            self.logger.debug(f"Published 'feedback_joint_velocity_msg' data: '{self.feedback_joint_velocity_msg.data}'")

            # Reinitializes previous_velocity to have the value of the current mean exoskeleton elbow joint velocity.
            self.previous_velocity = mean_elbow_joint_vel

            # Append data to respective lists.
            self.plot_time.append(time_now-self.program_start_time)
            self.plot_mean_elbow_joint_angle.append(mean_elbow_joint_angle)
            self.plot_j_vel.append(mean_elbow_joint_vel)
            self.plot_t_vel.append(self.t_vel)


    def manual_position_control_data_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Manual_position_control_data' using the subscription 'manual_position_control_data_subscription'.
        """

        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Sending data to microcontroller, with state set to be '/'.
        # This is part of the established encoding convention of the system.
        # When state is '/' the data is enterpreted as being a
        # target exoskeleton elbow joint angle by the microcontroller.
        # Currently the entirety of the position control system is running on the microcontroller.
        self.send_data(self.microcontroller, msg.data, seperator= "\n", state= "/")

        # This is an optional wait time between the system sends a message over serial
        # and recieves a message over serial.
        # The value DELAY_BETWEEN_SENDING_AND_RECEIVING is set in settings.json.
        time.sleep(self.DELAY_BETWEEN_SENDING_AND_RECEIVING)

        # Recieve serial data and cast it as integer.
        data = int(self.receive_data(self.microcontroller))

        # Log received serial data as debug level of importance.
        self.get_logger().debug(f"Received serial data: '{data}'")


    def map_range(self, x, in_min, in_max, out_min, out_max):
        """
        Function emulation the functionality of the Arduino IDE map() method.
        Maps a value x from one range [in_min:in_max] to another range [out_min:out_max].
        """

        # Mapping x from range [in_min:in_max] to range [out_min:out_max]
        output = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

        return output


#########################
########   MAIN   #######
#########################
        

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
GRAPH = handler.get_subkey_value("serial_communicator", "GRAPH")
AMOUNT_DATAPOINTS = handler.get_subkey_value("serial_communicator", "AMOUNT_DATAPOINTS")
LOG_DEBUG = handler.get_subkey_value("serial_communicator", "LOG_DEBUG")
LOG_LEVEL = handler.get_subkey_value("serial_communicator", "LOG_LEVEL")


# Initialize the rclpy library.
rclpy.init()

# Sets the logging level of importance. 
# When setting, one is setting the lowest level of importance one is interested in logging.
# Logging level is defined in settings.json
# Logging levels:
# - DEBUG
# - INFO
# - WARNING
# - ERROR
# - FATAL
# The eval method interprets a string as a command.
rclpy.logging.set_logger_level("serial_communicator", eval(LOG_LEVEL))

# Instance the Serial_Communicator node class.
serial_communicator = Serial_Communicator(SERIAL_PORT, BAUD_RATE, BYTESIZE, PARITY, STOPBITS, DELAY_BETWEEN_SENDING_AND_RECEIVING, RUNNING_AVERAGE_BUFFER_SIZE, LOG_DEBUG)

# This statement is true if the value GRAPH is true. This can be set in settings.json
# If true then a plot is made of the mean exoskeleton elbow joint velocity and angle, as well as the target velocity, over time.
if GRAPH:

    # Iteration counter
    iter = 0

    # This statement is true while the iteration counter is less than the value of AMOUNT_DATAPOINTS, which is set in settings.json.
    while iter < AMOUNT_DATAPOINTS:

        # Spinning the node once, so that the amount of node spins can be controlled.
        rclpy.spin_once(serial_communicator)

        # Incrementing the iteration counter by one.
        iter += 1

    # Plotting the mean exoskeleton elbow joint velocity and angle, as well as the target velocity, over time. 
    plt.plot(serial_communicator.plot_time, serial_communicator.plot_j_vel, serial_communicator.plot_time, serial_communicator.plot_mean_elbow_joint_angle, serial_communicator.plot_time, serial_communicator.plot_t_vel)
    plt.ylabel('Target velocity and computed runnning average joint velocity & joint angle')
    plt.grid(color='k', linestyle='-', linewidth=1)
    plt.legend()
    plt.show()

else:

    # Spin the node as normal.
    rclpy.spin(serial_communicator)


    
