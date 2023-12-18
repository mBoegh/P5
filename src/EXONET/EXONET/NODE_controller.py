from EXONET.EXOLIB import JSON_Handler
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16, Bool, Int64, Float32
import time
from time import perf_counter

import numpy as np
from simple_pid import PID

class Variables:
    """
    Class for initialising, setting and getting Variables.
    When instanced in global space, then every other scope can access the Variables.
    """

    def __init__(self, upper_bound, lower_bound):

        # Initialising parsed Variables.
        self.upper_bound = upper_bound  # deg
        self.lower_bound = lower_bound  # deg

        # Initialising class Variables.
        
        self.t_vel = 0  # Target exoskeleton elbow joint velocity
        self.t_pos = 0  # Target exoskeleton elbow joint angle
        self.current_pos = 0  # Current exoskeleton elbow joint angle

        # Variables that need to subscribe to the right stuff
        self.j_vel = 0 # Joint velocity
        self.elbow_joint_angle = 90 # Joint angle 
        self.prev_time = time.time()
        
        # Constants for the gravity compensation (need updates)
        self.g_acceleration = 9.82 # Gravitational acceleration
        self.exo_weight = 0.32 #kg
        self.av_arm_weight = 1 # kg
        self.av_payload_weight = 0.5 # kg
        self.shoulder_joint_angle = 45 # deg, should always be 0 since we don't know what it is
        self.cable_angle = 45 # The angle which the cable is attached to the exoskeleton
        self.lm1_length = 0.10 # meters
        self.l2_lenght = 0.225 # meters
        self.lm2_length = 0.10 # meters 
        self.spool_radius  = 0.025 # meters 

        # Constants for the spring compensation
        self.relaxed_spring_length = 0.11 # m
        self.spring_constant = 60 # N/m
        self.spring_arm_length = 0.07 # m
        self.spring_arm_bend_angle = 20 # deg
        self.upper_arm_construction_length = 0.185 # m
        

class Controller(Node):
    """
    This is the Controller node of the EXONET ROS2 network.
    The purpose of the Controller is handling all computations for the closed loop control systen.
    """

    def __init__(self, timer_period, stepwise, log_debug):

        # Initialising parsed Variables
        self.TIMER_PERIOD = timer_period
        self.STEPWISE = stepwise
        self.LOG_DEBUG = log_debug
        
        # Initialising class Variables
        self.prev_duty_cycle = 0
        self.prev_vel = 0

        self.prev_spring_acc = 0
        self.prev_gravity_acc = 0

        self.lower_inertia = (1/3) * (variables.av_arm_weight + variables.exo_weight) * np.square(variables.l2_lenght) #kg*m2
        self.payload_inertia = variables.av_payload_weight * np.square(variables.l2_lenght) #kg*m2

        self.toggle_EEG_parameter = False

        self.called_manual_velocity_control_data_topic_callback = False
        self.called_eeg_data_topic_callback = False
        self.called_feedback_topic_callback = False

        self.pid_state = True

        # Initialising variable msg as being of data type 'std_msgs.msg.Int8' imported as Int8
        # The message is loaded with data and published to a topic
        self.duty_cycle_msg = Int16()

        # Initialising variable msg as being of data type 'std_msgs.msg.Int8' imported as Int8
        # The message is loaded with data and published to a topic
        self.target_velocity_msg = Int16()

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('controller')

        # Initialising the PID class object from the simple_pid libary.
        # Parsed arguments are P constant, I constant and D constant, as well as the setpoint, which is the target value.        
        self.pi = PID(50, 20, 0, setpoint=variables.t_pos)

        # Limits the output of the PID controller to be in range -100 to 100.
        self.pi.output_limits = (-100, 100)

        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

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

        # Initialising a subscriber to the topic 'Manual_velocity_control_data'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_velocity_control_data_subscription = self.create_subscription(Int16, 'Manual_velocity_control_data', self.manual_velocity_control_data_topic_callback, 10)
        self.manual_velocity_control_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Feedback_joint_velocity'.
        # On this topic is expected data of type std_msgs.msg.Float32 which is imported as Float32.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_joint_velocity_subscription = self.create_subscription(Float32, 'Feedback_joint_velocity', self.feedback_joint_velocity_topic_callback, 10)
        self.feedback_joint_velocity_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Feedback_joint_angle'.
        # On this topic is expected data of type std_msgs.msg.Float32 which is imported as Float32.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.feedback_joint_angle_subscription = self.create_subscription(Float32, 'Feedback_joint_angle', self.feedback_joint_angle_topic_callback, 10)
        self.feedback_joint_angle_subscription

        # Initialising a publisher to the topic 'Velocity_motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.velocity_motor_signals_publisher = self.create_publisher(Int16, 'Velocity_motor_signals', 10)
        self.velocity_motor_signals_publisher  # prevent unused variable warning

        # Initialising a publisher to the topic 'Target_velocity'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.target_velocity_publisher = self.create_publisher(Int16, 'Target_velocity', 10)
        self.target_velocity_publisher  # prevent unused variable warning

        # Create a timer which periodically calls the specified callback function at a defined interval.
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)


    def map_range(self, x, in_min, in_max, out_min, out_max):
        """
        Function emulation the functionality of the Arduino IDE map() method.
        Maps a value x from one range [in_min:in_max] to another range [out_min:out_max].
        """

        # Mapping x from range [in_min:in_max] to range [out_min:out_max]
        output = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

        return output


    def eeg_toggle_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'EEG_toggle' using the subscription 'eeg_toggle_subscription'.
        """

        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Received data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is as a boolean value.
        toggle = msg.data

        # This statement is true if the received boolean value is True.
        # Toggles the toggle_EEG_parameter true thereby letting the system know that it shall react to the EEG data stream.
        if toggle == True:

            # Set the toggle_EEG_parameter true, thereby letting the system know that it shall react to the EEG data stream.
            self.toggle_EEG_parameter = True

            # Log that the system recognised that the user toggled the EEG switch in the GUI node, as debug level of importance.
            self.get_logger().debug(f"Toggled EEG True.")

        # This statement is true if the received boolean value is False.
        # Toggles the toggle_EEG_parameter false thereby letting the system know that it shall not react to the EEG data stream.
        elif toggle == False:

            # Set the toggle_EEG_parameter false, thereby letting the system know that it shall not react to the EEG data stream.
            self.toggle_EEG_parameter = False
        
            # Log that the system recognised that the user toggled the EEG switch in the GUI node to false, as debug level of importance.
            self.get_logger().debug(f"Toggled EEG False.")


        else:

            # Log that none of the above statements were true, meaning that the received message was unconventional, as warning level of importance.
            self.get_logger().warning(f"Unexpected message data on topic.")


    def eeg_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'EEG_data' using the subscription 'eeg_data_subscription'.
        """

        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: {msg.data}")

        # Raise the called_eeg_data_topic_callback flag, to indicate that the function has been called.
        # If this and the toggle_EEG_parameter flag are both true, then the timer_callback will start
        # computing the closed loop control system based on the eeg data stream and the exoskeleton feedback data.
        self.called_eeg_data_topic_callback = True

        # try / except statement used for catching errors, logging them and continuing with program execution.
        try:

            # This statement is true if the toggle_EEG_parameter flag is true.
            # If true then the topic message string is unpacked and segmented into
            # the mental command and the command power.
            # The command power is mapped to be in the range of the target velocity,
            # and then set as setpoint for the PID controller.
            if self.toggle_EEG_parameter == True:

                # Unpack the recieved message data.
                # The expected data is formatted as an integer from -100 to 100.
                data_string = msg.data

                # This statement is true if the data_string string includes at least one comma and at least one forward slash.
                # If true then string is shortened to be until and without the first forward slash in the string.
                # This is done to prevent multiple messages being present in the string at the same time, which would break the decoding procedure.
                # Then the mental command is found from the beginning of the string until and without the comma.
                # Then the command power is found from and without the comma to the end of the string.
                if "," in data_string and "/" in data_string:

                    # Find the index of the first forward slash in the string.
                    breaker_index = data_string.index("/")

                    # Reinitialize the data_string as the string up until and without the forward slash.
                    data_string = data_string[:breaker_index]

                    # Find the index of the first comma in the string.
                    seperator_index = data_string.index(",")

                    # Initialize the mental_command as the string up until and without the comma.
                    mental_command = data_string[:seperator_index]
                    
                    # Initialize the command power as the string from and without the comma to the end of the string interpreted as an integer.
                    command_power = int(data_string[seperator_index+1:])

                # This statement is true if the mental_command is determined to have value "Lift".
                # If true then the t_vel target velocity is reinitialized as being the command power
                # mapped from the range 0 - 100 to the range of the target velocity 0 - 40 deg/s.
                # Then the PID controller setpoint is set to be the t_vel.
                if mental_command == "Lift":

                    # Mapping the command power from the range 0 - 100 to the range of the target velocity 0 - 40 deg/s. 
                    variables.t_vel = self.map_range(command_power, 0, 100, 0, 40)

                    # Set the PID controller setpoint to be the t_vel target velocity.
                    controller.pi.setpoint = variables.t_vel

                    # Log the updated target joint velocity, as debug level of importance.
                    self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")

                # This statement is true if the mental_command is determined to have value "Drop".
                # If true then the t_vel target velocity is reinitialized as being the negative command power
                # mapped from the range -100 - 0 to the range of the target velocity -40 - 0 deg/s.
                # Then the PID controller setpoint is set to be the t_vel.
                elif mental_command == "Drop":

                    # Mapping the command power from the range -100 - 0 to the range of the target velocity -40 - 0 deg/s. 
                    variables.t_vel = self.map_range(-command_power, -100, 0, -40, 0)

                    # Set the PID controller setpoint to be the t_vel target velocity.
                    controller.pi.setpoint = variables.t_vel

                    # Log the updated target joint velocity, as debug level of importance.
                    self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")

                # This statement is true if the mental_command is determined to have value "Neutral".
                # If true then the t_vel target velocity is reinitialized as being zero.
                elif mental_command == "Neutral":

                    # Reinitializing the t_vel target velocity to be zero.
                    variables.t_vel = 0

                    # Set the PID controller setpoint to be the t_vel target velocity.
                    controller.pi.setpoint = variables.t_vel

                    # Log the updated target joint velocity, as debug level of importance.
                    self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")

                else:

                    # Logs that the segmented mental command is unrecognised, as warning level of importance.
                    self.get_logger().warning(f"Unexpected mental command in recieved EEG data.")

        except Exception as e:
            
            # Logs the exception formatted with the context that the received message did not follow the established encoding convention, as warning level of importance.
            self.get_logger().warning(f"Unexpected EEG data with error: {e}")


    def manual_velocity_control_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Manual_velocity_control_data' using the subscription 'manual_control_subscription'.
        """ 

        # Log received message data, as debug level of importance.
        self.get_logger().debug(f"Received data: '{msg.data}'")

        # Raise the called_manual_velocity_control_data_topic_callback flag, to indicate that the function has been called.
        # If this is true and the toggle_EEG_parameter flag is not true, then the timer_callback will start
        # computing the closed loop control system based on the manually generated target velocitiy data and the exoskeleton feedback data.
        self.called_manual_velocity_control_data_topic_callback = True

        # This statement is true if the toggle_EEG_parameter flag is false.
        # If true then the t_vel target velocity is reinitialized to be the value of the recieved topic message data.
        # Then the PID controller setpoint is set to be the t_vel.
        if self.toggle_EEG_parameter == False:

            # Unpack the recieved message data.
            # The expected data is as a integer value.
            variables.t_vel = msg.data

            # Set the PID controller setpoint to be the t_vel target velocity.
            controller.pi.setpoint = variables.t_vel

            # Log the updated target joint velocity, as debug level of importance.
            self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")


    def feedback_joint_velocity_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Feedback_joint_velocity' using the subscription 'feedback_joint_velocity_subscription'.
        """

        # Log received message data, as debug level of importance.
        self.get_logger().debug(f"Recieved data '{msg.data}'")

        # Reinitialize the j_vel joint velocity with the received message data.
        # The expected data is as a float32 value.
        variables.j_vel = msg.data

        # Log the change to j_vel joint velocity, as debug level of importance.
        self.get_logger().debug(f"Updated current joint velocity with value: {variables.j_vel}")


    def feedback_joint_angle_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Feedback_joint_angle' using the subscription 'feedback_joint_angle_subscription'.
        """

        # Log received message data, as debug level of importance.
        self.get_logger().debug(f"Recieved data '{msg.data}'")


        # Reinitialize the elbow_joint_angle joint angle with the received message data.
        # The expected data is as a float32 value.
        variables.elbow_joint_angle = msg.data

        # Log the change to elbow_joint_angle joint angle, as debug level of importance.
        self.get_logger().debug(f"Updated current joint angle with value: {variables.elbow_joint_angle}")

        # This statement is true if the current exoskeleton elbow joint angle is equal to or greater than the greatest acceptable angle whilst the target velocity is greater than zero.
        # If true, that means that the exoskeleton is leaving its elbow joint angle limits.
        # Therefore the edge_guard function is called.
        if variables.elbow_joint_angle >= variables.upper_bound and variables.t_vel > 0:
            self.edge_guard()

        # This statement is true if the current exoskeleton elbow joint angle is equal to or less than the lowest acceptable angle whilst the target velocity is less than zero.
        # If true, that means that the exoskeleton is leaving its elbow joint angle limits.
        # Therefore the edge_guard function is called.    
        elif variables.elbow_joint_angle <= variables.lower_bound and variables.t_vel < 0: 
            self.edge_guard()


    def edge_guard(self):
        """
        Stops the motor if a movement will exceed the angular limits of the exoskeleton.
        """

        # Log that the exoskeleton has exceeded the software limits of the elbow joint angle and the updated target joint velocity, as debug level of importance.
        self.get_logger().debug(f"Software joint limits exceed.")
        
        # Reinitializing the t_vel target velocity to be zero.
        variables.t_vel = 0

        # Set the PID controller setpoint to be the t_vel target velocity.
        self.pi.setpoint = variables.t_vel
        
        # Log the updated target joint velocity, as debug level of importance.
        self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")

        # Reinitialize the prev_duty_cycle to zero.
        self.prev_duty_cycle = 0
        
        # Log that the prev_duty_cycle is reset to zero, as debug level of importance.
        self.get_logger().debug(f"Reset previous duty cycle: {self.prev_duty_cycle}")

        # Loads the duty_cycle_msg message with a zero duty cycle.
        self.duty_cycle_msg.data = 0
        
        # Publishes the duty cycle to the topic /Velocity_motor_signals.
        self.velocity_motor_signals_publisher.publish(self.duty_cycle_msg)
        
        # Log the published duty cycle, as debug level of importance.
        self.get_logger().debug(f"Published duty cycle: {self.duty_cycle_msg.data}")


    def timer_callback(self):
        """
        Function called at specific time interval, specified in 'settings.json'.
        This is where the closed loop control system is computed.
        """

        # This statement is true if either the called_manual_velocity_control_data_topic_callback flag is true while the toggle_EEG_parameter flag is not true,
        # or if the called_eeg_data_topic_callback flag is true whilst the toggle_EEG_parameter flag is true.
        # If true then the closed loop cotnrol system is computed.
        if (self.called_manual_velocity_control_data_topic_callback and not self.toggle_EEG_parameter) or (self.called_eeg_data_topic_callback and self.toggle_EEG_parameter):

            # Log the variables used in the controller, as debug level of importance.
            self.get_logger().debug(f"""VARIABLES USED IN CONTROLLER:
            - Target velocity: {variables.t_vel}
            - Joint velocity: {variables.j_vel}
            - Elbow joint angle: {variables.elbow_joint_angle}""")

            # This statement is true if the current t_vel target velocity is zero.
            # If true then the PID controllers error build up is reset to zero.
            if variables.t_vel == 0:

                # Sets the PID to not compute new values when it is called (essentially turning it off).
                self.pi.auto_mode = False

                # Sets the PID to compute new values when it is called.
                # Also sets the last_output to zero.
                # Essentially turns on the PID and makes sure that there is no carried over error from the last output.
                self.pi.set_auto_mode(True, last_output= 0)

            # Dynamic calculations for spring compensation
            spring_arm_angle =  variables.elbow_joint_angle + variables.spring_arm_bend_angle # deg

            tense_spring_length = np.sqrt(variables.spring_arm_length**2 + variables.upper_arm_construction_length**2 
                                         - 2 * variables.spring_arm_length*variables.upper_arm_construction_length * np.cos(np.deg2rad(spring_arm_angle))) # m

            spring_angle = np.rad2deg(np.arccos((tense_spring_length**2 + variables.upper_arm_construction_length**2 
                                                + variables.spring_arm_length**2) / (2 * tense_spring_length * variables.spring_arm_length))) # deg
            
            spring_force_vector = (tense_spring_length - variables.relaxed_spring_length) * variables.spring_constant # N
            
            spring_tangential_force_component = np.sin(np.deg2rad(spring_force_vector)) * spring_force_vector # N
            
            spring_compensation_torque = spring_tangential_force_component * variables.spring_arm_length # Nm
            
            current_time = time.time()
            
            spring_acc = spring_compensation_torque / (self.lower_inertia + self.payload_inertia) #((1.65 *gravity_compensation_torque + 10.24)/12)*100
            spring_vel = (spring_acc + self.prev_spring_acc) * 0.5  * (current_time - variables.prev_time)

            self.prev_spring_acc = spring_acc

            # Log the results from the dynamic calculations for spring compensation, as debug level of importance.
            self.get_logger().debug(
                f"Spring Compensation Calculations:"
                f"\n- spring_arm_angle: {spring_arm_angle}"
                f"\n- tense_spring_length: {tense_spring_length}"
                f"\n- spring_angle: {spring_angle}"
                f"\n- spring_force_vector: {spring_force_vector}"
                f"\n- spring_tangential_force_component: {spring_tangential_force_component}"
                f"\n- spring_compensation_torque: {spring_compensation_torque}"
            )

            # Dynamic calculations for gravity compensation.
            fg2 = (variables.av_arm_weight + variables.exo_weight) * variables.g_acceleration
            fgp = variables.av_payload_weight * variables.g_acceleration

            f2 = np.cos(np.deg2rad((variables.elbow_joint_angle+90) - variables.shoulder_joint_angle)) * (variables.l2_lenght / 2) * fg2
            fp = np.cos(np.deg2rad((variables.elbow_joint_angle+90) - variables.shoulder_joint_angle)) * variables.l2_lenght * fgp

            torque_joint = f2 * (variables.l2_lenght / 2) + fp * variables.l2_lenght

            # Log the results from the dynamic calculations for gravity compensation, as debug level of importance.
            self.get_logger().debug(
                f"Gravity Compensation Calculations:"
                f"\n- fg2: {fg2}"
                f"\n- fgp: {fgp}"
                f"\n- f2: {f2}"
                f"\n- fp: {fp}"
                f"\n- Torque Joint: {torque_joint}"
            )

            # Dynamic calculations for cable dynamics.
            cable_length = np.sqrt(np.square(variables.lm1_length)+np.square(variables.lm2_length)-2*variables.lm1_length*variables.lm2_length*np.cos(np.deg2rad(variables.elbow_joint_angle)))
            variables.cable_angle = np.rad2deg(np.arccos(np.divide((np.square(cable_length) + np.square(variables.lm2_length) - np.square(variables.lm1_length)), (2*cable_length*variables.lm2_length))))

            # Force cable components 1 and 2.
            fc1 = torque_joint / variables.lm2_length
            fc2 = np.tan(np.deg2rad((np.pi / 2) - variables.cable_angle)) * fc1

            # Find the cable force and calculate the motor torque.
            fc = np.sqrt(fc1**2 + fc2**2)

            # Log the results from the dynamic calculations for cable dynamics, as debug level of importance.
            self.get_logger().debug(
                f"Force Cable Calculations:"
                f"\n- cable_length: {cable_length}"
                f"\n- cable_angle: {variables.cable_angle}"
                f"\n- fc1: {fc1}"
                f"\n- fc2: {fc2}"
                f"\n- fc: {fc}"
            )

            # Compute the torque necessary to compensate for gravity.
            gravity_compensation_torque = fc * variables.spool_radius

            # Log the result from the dynamic calculation for the gravity compensation torque, as debug level of importance.
            self.get_logger().debug(
                f"Torque Motor Calculation:"
                f"\n- Gravity compensation torque: {gravity_compensation_torque}"
            )

            gravity_acc = gravity_compensation_torque / (self.lower_inertia + self.payload_inertia) 
            gravity_vel = (gravity_acc + self.prev_gravity_acc) * 0.5  * (current_time - variables.prev_time)

            self.prev_gravity_acc = gravity_acc

            angle_comp = 0
            if variables.t_vel < 0:
                angle_comp = (variables.elbow_joint_angle-45)/2
            elif variables.t_vel > 0:
                angle_comp = (180-variables.elbow_joint_angle-110)/4
            
            # Angle compensation
            variables.t_pos = variables.elbow_joint_angle+(variables.t_vel + (spring_vel + gravity_vel) + self.prev_vel - angle_comp) * 0.5  * (current_time - variables.prev_time)

            self.get_logger().debug(f"Target Position: {variables.t_pos}")
            self.get_logger().debug(f"Current Position: {variables.elbow_joint_angle}")
            
            # Set the PID controller setpoint to be the t_vel target velocity.
            controller.pi.setpoint = variables.t_pos

            # Compute the duty cycle from PID regulating the current exoskeleton elbow joint angle.
            duty_cycle = self.pi(variables.elbow_joint_angle)
            
            self.prev_vel = variables.t_vel

            # Log controller calculations
            self.get_logger().debug(
                f"Controller Calculations:"
                f"\n- Duty Cycle: {duty_cycle}"
            )

            self.prev_duty_cycle = duty_cycle
            variables.prev_time = current_time

            # Reinitializing duty_cycle to be itself cast as integer, esentially rounding to nearest whole number.
            duty_cycle = int(duty_cycle)

            # Log the rounded duty cycle, as debug level of importance.
            self.get_logger().debug(f"Duty cycle rounded: {duty_cycle}")

            # Loads the duty_cycle_msg message with the computed duty_cycle data.
            self.duty_cycle_msg.data = duty_cycle

            # This statement is true if the current PID controller setpoint, eg. target, is zero.
            # If true then the duty_cycle_msg message is loaded with zero instead of the computed duty cycle.
            if self.pi.setpoint == 0:

                # Loads the duty_cycle_msg message with zero.
                self.duty_cycle_msg.data = 0

            # Log the data content of the duty_cycle_msg message, as debug level of importance.
            self.get_logger().debug(f"Duty cycle message data: {self.duty_cycle_msg.data}")

            # Loads the target_velocity_msg message with the current t_vel target velocity.
            self.target_velocity_msg.data = variables.t_vel

            # Publishes the target_velocity_msg message using target_velocity_publisher ont topic 'Target_velocity'.
            self.target_velocity_publisher.publish(self.target_velocity_msg)

            # Publish duty_cycle_msg using velocity_motor_signals_publisher on topic 'Velocity_motor_signals'
            self.velocity_motor_signals_publisher.publish(self.duty_cycle_msg)


            # Logs the published t_vel target velocity data, as debug level of importance.
            self.get_logger().debug(f"Published 'target_velocity_publisher' data: '{self.target_velocity_msg.data}'")

            # Logs the published duty cycle data, as debug level of importance.
            self.get_logger().debug(f"Published 'velocity_motor_signals_publisher' data: '{self.duty_cycle_msg.data}'")

            # This statement is true if the STEPWISE variable is true, which is set in settings.json.
            # If true then the called_manual_velocity_control_data_topic_callback flag and the called_eeg_data_topic_callback flag are both lowered.
            # This halts the system until a new message is recieved again, meaning that when manually submitting target velocities 
            # the controller will only when fed a new target, ie. stepwise.
            if self.STEPWISE:
                
                # Lowers the called_manual_velocity_control_data_topic_callback flag.
                self.called_manual_velocity_control_data_topic_callback = False

                # Lowers the called_eeg_data_topic_callback flag.
                self.called_eeg_data_topic_callback = False
        

####################
######  MAIN  ######
####################


    
# Path for 'settings.json' file
json_file_path = ".//src//EXONET//EXONET//settings.json"

# Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
handler = JSON_Handler(json_file_path)

# Get settings from 'settings.json' file
TIMER_PERIOD = handler.get_subkey_value("controller", "TIMER_PERIOD")
STEPWISE = handler.get_subkey_value("controller", "STEPWISE")
UPPER_BOUND = handler.get_subkey_value("controller", "UPPER_BOUND")
LOWER_BOUND = handler.get_subkey_value("controller", "LOWER_BOUND")
LOG_DEBUG = handler.get_subkey_value("controller", "LOG_DEBUG")
LOG_LEVEL = handler.get_subkey_value("controller", "LOG_LEVEL")


variables = Variables(UPPER_BOUND, LOWER_BOUND)

# Initialize the rclpy library
rclpy.init()

rclpy.logging.set_logger_level("controller", eval(LOG_LEVEL))

# Instance the serverTCP class
controller = Controller(TIMER_PERIOD, STEPWISE, LOG_DEBUG)

# Begin looping the node
rclpy.spin(controller)
    


    