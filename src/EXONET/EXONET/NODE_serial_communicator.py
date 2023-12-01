"""
TO DO:
 - Test live with Emotiv Epoc X
"""

from EXONET.EXOLIB import JSON_Handler, serial2arduino, RunningAverage, LiveLFilter

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int64, Float32, Int16, String

import numpy as np
import matplotlib.pyplot as plt

import scipy.signal

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
        self.plot_filtered_data = []
        self.plot_elbow_joint_angle = []
        self.plot_mean_elbow_joint_angle = []
        self.plot_j_vel = []


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
        self.velocity_motor_signals_subscription = self.create_subscription(Int16, 'Velocity_motor_signals', self.motor_signals_topic_callback, 10)
        self.velocity_motor_signals_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Manual_position_control_data'.
        # On this topic is expected data of type std_msgs.msg.UInt16 which is imported as UInt16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_position_control_data_subscriber = self.create_subscription(UInt16, 'Manual_position_control_data', self.manual_position_control_data_callback, 10)
        self.manual_position_control_data_subscriber

        # Initialising a subscriber to the topic 'Manual_position_control_data'.
        # On this topic is expected data of type std_msgs.msg.UInt16 which is imported as UInt16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_input_velocity_control_data_subscriber = self.create_subscription(Int16, 'Manual_input_velocity_control_data', self.motor_signals_topic_callback, 10)
        self.manual_input_velocity_control_data_subscriber

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
        self.get_logger().info(f"Recieved topic data: '{msg.data}'")

        duty_cycle = msg.data

        serial_message = f"0{duty_cycle + 1000}"

        # Sending data to Arduino
        self.send_data(self.arduino, serial_message, seperator="\n")

       # time.sleep(self.DELAY_BETWEEN_SENDING_AND_RECEIVING)

        # Load feedback_msg with returned data 
        data = int(self.receive_data(self.arduino))

        self.get_logger().debug(f"Received serial data: '{data}'")

        filtered_data = data # livel_filter(data)

        if self.first_feedback:
            self.time0 = time.time()
            self.elbow_joint_angle_zero = self.map_range(1023-filtered_data, 0, 1023, -30, 210) # Joint angle 

            self.running_average = RunningAverage(RUNNING_AVERAGE_BUFFER_SIZE, self.elbow_joint_angle_zero)

            self.first_feedback = False

        else:

            time_now = time.time()

            elbow_joint_angle_now = self.map_range(1023-filtered_data, 0, 1023, -30, 210) # Joint angle 
        
            # Compute running average using the RunningAverage object of the EXOLIB library with buffersize n defined in settings.json
            self.running_average.add_data_point(elbow_joint_angle_now)
            mean_elbow_joint_angle = self.running_average.get_average()
        
            time_diff = time_now - self.time0

            self.time0 = time_now
        
            elbow_joint_angle_diff = mean_elbow_joint_angle - self.elbow_joint_angle_zero

            self.elbow_joint_angle_zero = mean_elbow_joint_angle

            self.data0 = filtered_data        
            
            j_vel = elbow_joint_angle_diff / time_diff  # Joint velocity

            # # Compute running average using the RunningAverage object of the EXOLIB library with buffersize n defined in settings.json
            # self.add_data_point(j_vel)
            # mean_j_vel = self.get_average()

            self.feedback_joint_velocity_msg.data = float(j_vel)
            self.feedback_joint_angle_msg.data = float(mean_elbow_joint_angle)

            self.get_logger().debug(f"Computed 'j_vel' feedback data: '{j_vel}'")
            self.get_logger().debug(f"Computed 'elbow_joint_angle' feedback data: '{mean_elbow_joint_angle}'")

            if not self.previous_velocity == 0 and j_vel > 10:
                self.feedback_joint_angle_publisher.publish(self.feedback_joint_angle_msg)
                self.feedback_joint_velocity_publisher.publish(self.feedback_joint_velocity_msg)
            else:
                self.feedback_joint_velocity_msg.data = float(0)

                self.feedback_joint_angle_publisher.publish(self.feedback_joint_angle_msg)
                self.feedback_joint_velocity_publisher.publish(self.feedback_joint_velocity_msg)

            self.previous_velocity = j_vel

            self.plot_data.append(data)
            self.plot_filtered_data.append(filtered_data)
            self.plot_elbow_joint_angle.append(elbow_joint_angle_now)
            self.plot_mean_elbow_joint_angle.append(mean_elbow_joint_angle)
            self.plot_j_vel.append(j_vel)

    def manual_position_control_data_callback(self, msg):

        # Log info
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Sending data to Arduino
        self.send_data(self.arduino, msg.data, seperator= "\n", state= "/")

       # time.sleep(self.DELAY_BETWEEN_SENDING_AND_RECEIVING)

        # Load feedback_msg with returned data 
        data = int(self.receive_data(self.arduino))

        self.get_logger().debug(f"Received serial data: '{data}'")


    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

####################
######  MAIN  ######
####################

def plot_signals_and_spectrum(time, signal, cutoff_frequency, filtered_signal, fs):
    """
    Plot the time-domain and frequency-domain representations of a signal.

    Parameters:
    - time: Time vector of the signal.
    - signal: Original signal.
    - filtered_signal: Signal after applying the lowpass filter.
    - fs: Sampling frequency.
    """
    plt.figure(figsize=(12, 6))

    # Plot the time-domain signal
    plt.subplot(3, 1, 1)
    plt.plot(time, signal, label='Original Signal')
    plt.plot(time, filtered_signal, label='Filtered Signal', linewidth=2)
    plt.title('Time Domain')
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.legend()

    # Plot the frequency-domain representation using FFT
    plt.subplot(3, 1, 2)
    fft_freq = np.fft.fftfreq(len(signal), 1/fs)
    fft_signal = np.fft.fft(signal)
    plt.plot(fft_freq, np.abs(fft_signal), label='Original Signal')
    
    fft_filtered_signal = np.fft.fft(filtered_signal)
    plt.plot(fft_freq, np.abs(fft_filtered_signal), label='Filtered Signal', linewidth=2)
    
    plt.title('Frequency Domain (FFT)')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Magnitude')
    plt.legend()

    # Plot the frequency response of the filter
    plt.subplot(3, 1, 3)
    b, a = butter_lowpass(cutoff_frequency, fs, order=4)
    w, h = freqz(b, a, worN=8000)
    plt.plot(0.5 * fs * w / np.pi, np.abs(h), 'b', label='Lowpass Filter')
    plt.plot(cutoff_frequency, 0.5 * np.sqrt(2), 'ko')
    plt.axvline(cutoff_frequency, color='k')
    plt.xlim(0, 0.5 * fs)
    plt.title('Lowpass Filter Frequency Response')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Gain')
    plt.legend()

    plt.tight_layout()
    plt.show()

def butter_lowpass(cutoff_freq, fs, order=4):
    """
    Design a lowpass Butterworth filter.

    Parameters:
    - cutoff_freq: Cutoff frequency of the filter.
    - fs: Sampling frequency.
    - order: Filter order (default is 4).

    Returns:
    - b: Numerator coefficients of the filter.
    - a: Denominator coefficients of the filter.
    """
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    # Design the Butterworth filter
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a



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
LOG_DEBUG = handler.get_subkey_value("serial_communicator", "LOG_DEBUG")
LOG_LEVEL = handler.get_subkey_value("serial_communicator", "LOG_LEVEL")


# Initialize the rclpy library
rclpy.init()

rclpy.logging.set_logger_level("serial_communicator", eval(LOG_LEVEL))


# Butterworth low-pass filter with frequency cutoff at 125 Hz
b, a = scipy.signal.iirfilter(4, Wn=49, fs=100, btype="low", ftype="butter")

livel_filter = LiveLFilter(b, a)

# Instance the serverTCP class
serial_communicator = Serial_Communicator(SERIAL_PORT, BAUD_RATE, BYTESIZE, PARITY, STOPBITS, DELAY_BETWEEN_SENDING_AND_RECEIVING, RUNNING_AVERAGE_BUFFER_SIZE, LOG_DEBUG)

#rclpy.spin(serial_communicator)

iter = 0
while iter < 600:
    # Begin looping the node
    rclpy.spin_once(serial_communicator)
    iter += 1

# plt.plot(serial_communicator.plot_data)
# plt.ylim((0,1023))
# plt.show()

# plt.plot(serial_communicator.plot_filtered_data)
# plt.ylim((0,1023))
# plt.show()

# plt.plot(serial_communicator.plot_elbow_joint_angle)
# plt.ylabel('Elbow joint angle')
# plt.ylim((130,300))
# plt.show()

# plt.plot(serial_communicator.plot_j_vel)
# plt.ylabel('Computed joint velocity')
# plt.show()

plt.plot(range(599), serial_communicator.plot_j_vel, range(599), serial_communicator.plot_mean_elbow_joint_angle)
plt.ylabel('Computed runnning average joint velocity')
plt.grid(color='k', linestyle='-', linewidth=1)
plt.legend()
plt.show()

# datas = [serial_communicator.plot_data, serial_communicator.plot_elbow_joint_angle, serial_communicator.plot_j_vel, serial_communicator.plot_mean_j_vel]

# # Define cutoff frequency for the lowpass filter
# cutoff_frequency = 50  # Adjust this based on your requirements

# fs = 999  # Sample rate, change it to your actual sample rate
# t = np.linspace(0, 1, fs, endpoint=False)  # Time vector

# for data in datas:

#     # Apply the lowpass filter to the data
#     b, a = butter_lowpass(cutoff_frequency, fs)
#     filtered_data = lfilter(b, a, data)

#     # Plot the time-domain and frequency-domain representations
#     plot_signals_and_spectrum(t, data, cutoff_frequency, filtered_data, fs)


    
