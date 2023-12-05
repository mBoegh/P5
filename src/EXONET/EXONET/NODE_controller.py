"""
TO DO:
 - Create controll system
"""

from EXONET.EXOLIB import JSON_Handler
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16, Bool, Int64, Float32

import numpy as np
from simple_pid import PID

class Variables():

    def __init__(self):

        self.t_vel = 0 # Target velocity

        # Variables that need to subscribe to the right stuff
        self.j_vel = 0 # Joint velocity
        self.elbow_joint_angle = 90 # Joint angle 
        
        # Constants for the gravity compensation (need updates)
        self.g_acceleration = 9.82 # Gravitational acceleration
        self.exo_weight = 1 #kg
        self.av_arm_weight = 1 # kg
        self.av_payload_weight = 0.5 # kg
        self.shoulder_joint_angle = 0 # kg, should always be 0 since we don't know what it is
        self.cable_angle = 45 # The angle which the cable is attached to the exoskeleton
        self.l2_lenght = 0.225 # meters
        self.lm2_length = 0.10 # meters 
        self.spool_radius  = 0.025 # meters 

        # Constants for the spring compensation
        self.relaxed_spring_length = 0.11 # m
        self.spring_constant = 60 # N/m
        self.spring_arm_length = 0.7 # m
        self.spring_arm_bend_angle = 20 # deg
        self.upper_arm_construction_length = 0.185 # m
        

class Controller(Node):
    """
    This is the Controller node of the EXONET ROS2 network.
    Takes argument(s):
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, timer_period, stepwise, log_debug):

        # Initialising variables
        self.TIMER_PERIOD = timer_period
        self.STEPWISE = stepwise
        self.LOG_DEBUG = log_debug

        # D should always be 0, Don't change setpoint!!! 
        self.pi = PID(1, 1, 0, setpoint=variables.t_vel) # setpoint=1
        self.pi.output_limits = (-100, 100)
        self.prev_duty_cycle = 0

        self.toggle_EEG_parameter = False

        self.called_manual_control_data_topic_callback = False
        self.called_eeg_data_topic_callback = False
        self.called_feedback_topic_callback = False

        self.pid_state = True

        self.msg = Int16()

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('controller')

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

        # Initialising a subscriber to the topic 'Manual_control_data'.
        # On this topic is expected data of type std_msgs.msg.Int8 which is imported as Int8.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_velocity_control_data_subscription = self.create_subscription(Int16, 'Manual_velocity_control_data', self.manual_velocity_control_data_topic_callback, 10)
        self.manual_velocity_control_data_subscription  # prevent unused variable warning

        # Initialising a publisher to the topic 'Motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int8 which is imported as Int8.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.velocity_motor_signals_publisher = self.create_publisher(Int16, 'Velocity_motor_signals', 10)
        self.velocity_motor_signals_publisher  # prevent unused variable warning

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

        # Create a timer which periodically calls the specified callback function at a defined interval.
        # Initialise timer_counter as zero. This is iterated on each node spin
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)
        self.timer_counter = 0


    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


    def eeg_toggle_topic_callback(self, msg):
        
        toggle = msg.data

        if toggle == True:
            self.get_logger().debug(f"Toggled EEG True")

            self.toggle_EEG_parameter = True
        
        elif toggle == False:
            self.get_logger().debug(f"Toggled EEG False")

            self.toggle_EEG_parameter = False
        
        else:
            self.get_logger().warning(f"Unexpected message data on topic.")


    def eeg_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'manual_data_subscription'
        """

        self.called_eeg_data_topic_callback = True

        if self.toggle_EEG_parameter == True:
            data_string = msg.data

            if "," in data_string:
                seperator_index = data_string.index(",")
            
                mental_command = data_string[:seperator_index]
                
                command_power = data_string[seperator_index+1:]

            if mental_command == "Lift":
                variables.t_vel = self.map_range(command_power, 0, 100, 0, 40)

                controller.pi.setpoint = variables.t_vel

            
            elif mental_command == "Drop":
                variables.t_vel = self.map_range(-command_power, -100, 0, -40, 0)

                controller.pi.setpoint = variables.t_vel

            else:
                self.get_logger().warning(f"Unexpected mental command in recieved EEG data.")

        

    def manual_velocity_control_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'manual_control_subscription'
        """ 

        self.called_manual_control_data_topic_callback = True

        if self.toggle_EEG_parameter == False:
            variables.t_vel = msg.data # Target velocity

            controller.pi.setpoint = variables.t_vel

            self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")


    def feedback_joint_velocity_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'feedback_joint_velocity_subscription'
        """

        self.get_logger().debug(f"Recieved data '{msg.data}'")

        variables.j_vel = msg.data

        self.get_logger().debug(f"Updated current joint velocity with value: {variables.j_vel}")


    def feedback_joint_angle_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'feedback_joint_angle_subscription'
        """

        def edge_guard():
            variables.t_vel = 0

            # self.pid_state == False
            # self.pi.auto_mode = False
            # self.get_logger().debug(f"PI controller off")

            self.get_logger().debug(f"Physical joint limits exceed. Target velocity: {variables.t_vel}")

            self.pi.setpoint = variables.t_vel
            
            self.get_logger().debug(f"Updated controller setpoint: {self.pi.setpoint}")

            self.prev_duty_cycle = 0

            self.get_logger().debug(f"Reset previous dutycycle: {self.prev_duty_cycle}")

            self.msg.data = 0

            self.velocity_motor_signals_publisher.publish(self.msg)

            self.get_logger().debug(f"Published dutycycle: {self.msg.data}")

        self.get_logger().debug(f"Recieved data '{msg.data}'")

        variables.elbow_joint_angle = msg.data

        self.get_logger().debug(f"Updated current joint angle with value: {variables.elbow_joint_angle}")

        if variables.elbow_joint_angle >= 120 and variables.t_vel > 0:
            edge_guard()

        elif variables.elbow_joint_angle <= 50 and variables.t_vel < 0: 
            edge_guard()
        
        # elif self.pid_state == False:
        #     self.pid_state == True
        #     self.pi.set_auto_mode(True, last_output=0.0)
        #     self.get_logger().debug(f"PI controller restarted")
            


    def timer_callback(self):
        
        self.get_logger().debug(f"Start of {self.timer_callback.__name__}")

        ## Closed loop control system ##
        if (self.called_manual_control_data_topic_callback and not self.toggle_EEG_parameter) or (self.called_eeg_data_topic_callback and self.toggle_EEG_parameter):

            self.get_logger().debug(f"Beggining of closed loop control system")

            # Log the variables used in the controller
            self.get_logger().debug(f"""VARIABLES USED IN CONTROLLER:
            - Target velocity: {variables.t_vel}
            - Joint velocity: {variables.j_vel}
            - Elbow joint angle: {variables.elbow_joint_angle}""")


            # Dynamic calculations for spring compensation
            spring_arm_angle = 180 - variables.elbow_joint_angle + variables.spring_arm_bend_angle # deg

            tense_spring_length = np.sqrt(variables.spring_arm_length**2 + variables.upper_arm_construction_length**2 - 2 * variables.spring_arm_length*variables.upper_arm_construction_length * np.cos(np.deg2rad(spring_arm_angle))) # m

            spring_angle = np.rad2deg(np.arccos((tense_spring_length**2 + variables.upper_arm_construction_length**2 + variables.spring_arm_length**2) / (2 * tense_spring_length * variables.spring_arm_length))) # deg
            
            spring_force_vector = (tense_spring_length - variables.relaxed_spring_length) * variables.spring_constant # N
            
            spring_tangential_force_component = np.sin(np.deg2rad(spring_force_vector)) * spring_force_vector # N
            
            spring_compensation_torque = spring_tangential_force_component * variables.spring_arm_length # Nm

            spring_compensation_duty_cycle = 3.301 * spring_compensation_torque + 10.24 # PWM Duty cycle signal

            self.get_logger().debug(
                f"Spring Compensation Calculations:"
                f"\n- spring_arm_angle: {spring_arm_angle}"
                f"\n- tense_spring_length: {tense_spring_length}"
                f"\n- spring_angle: {spring_angle}"
                f"\n- spring_force_vector: {spring_force_vector}"
                f"\n- spring_tangential_force_component: {spring_tangential_force_component}"
                f"\n- spring_compensation_torque: {spring_compensation_torque}"
                f"\n- spring_gravity_compensation_duty_cycle: {spring_compensation_duty_cycle}"
            )




            # Dynamic calculations for gravity compensation
            fg2 = (variables.av_arm_weight + variables.exo_weight) * variables.g_acceleration
            fgp = variables.av_payload_weight * variables.g_acceleration

            f2 = np.cos(np.deg2rad(variables.elbow_joint_angle - variables.shoulder_joint_angle)) * (variables.l2_lenght / 2) * fg2
            fp = np.cos(np.deg2rad(variables.elbow_joint_angle - variables.shoulder_joint_angle)) * variables.l2_lenght * fgp

            torque_joint = f2 * (variables.l2_lenght / 2) + fp * variables.l2_lenght

            # Log gravity compensation calculations
            self.get_logger().debug(
                f"Gravity Compensation Calculations:"
                f"\n- fg2: {fg2}"
                f"\n- fgp: {fgp}"
                f"\n- f2: {f2}"
                f"\n- fp: {fp}"
                f"\n- Torque Joint: {torque_joint}"
            )

            # Force cable components 1 and 2
            fc1 = torque_joint / variables.lm2_length
            fc2 = np.tan(np.deg2rad((np.pi / 2) - variables.cable_angle)) * fc1

            # Log force cable calculations
            self.get_logger().debug(
                f"Force Cable Calculations:"
                f"\n- fc1: {fc1}"
                f"\n- fc2: {fc2}"
            )

            # Force cable
            fc = np.sqrt(fc1**2 + fc2**2)

            torque_motor = fc * variables.spool_radius

            # Log torque motor calculation
            self.get_logger().debug(
                f"Torque Motor Calculation:"
                f"\n- Torque Motor: {torque_motor}"
            )

           # gravity_compensation_duty_cycle = 10.52 * torque_motor + 7.173  # function for converting torque to volt
            gravity_compensation_duty_cycle = 3.301 *torque_motor + 10.24

            # Log duty cycle calculations
            self.get_logger().debug(
                f"Duty Cycle Calculations:"
                f"\n- Compensation Duty Cycle: {gravity_compensation_duty_cycle}"
            )

            #gravity_acc_compensation = fc * (av_arm_weight+av_payload_weight+exo_weight)

            #time = time.time() - self.start_time
            #v = self.v0 + gravity_acc_compensation * time
            #self.time = time.time()
            #self.v0 = v 

            # The controller
            regulator = self.pi(variables.j_vel)
            duty_cycle = regulator # + gravity_compensation_duty_cycle + spring_compensation_duty_cycle
            
            if duty_cycle > 100:
                duty_cycle = 100
            if duty_cycle < -100:
                duty_cycle = -100

            # Log controller calculations
            self.get_logger().info(
                f"Controller Calculations:"
                f"\n- Control: {regulator}"
                f"\n- Duty Cycle: {duty_cycle}"
                f"\n- Previous duty cycle: {self.prev_duty_cycle}"
            )

            self.prev_duty_cycle = duty_cycle

            # The alternative controller
            # error = j_vel-t_vel
            # control = self.pi(error)
            # rpm = control/(2*np.pi/60)
            # volt = 4.7714*1.02**rpm
            
            # Load msg with duty cycle data
            duty_cycle = int(duty_cycle)

            self.get_logger().debug(f"Duty cycle rounded: {duty_cycle}")

            self.msg.data = duty_cycle

            if self.pi.setpoint == 0:
                self.msg.data = 0

            self.get_logger().debug(f"Duty cycle message data: {self.msg.data}")

            # Publish msg using velocity_motor_signals_publisher on topic 'Velocity_motor_signals'
            self.velocity_motor_signals_publisher.publish(self.msg)

            # Log info
            self.get_logger().info(f"Published data: '{self.msg.data}'")

            # Iterate timer
            self.timer_counter += 1

            self.get_logger().debug(f"End of closed loop control system")

            if self.STEPWISE:
                self.called_manual_control_data_topic_callback = False
                self.called_eeg_data_topic_callback = False
        
        self.get_logger().debug(f"End of {self.timer_callback.__name__}")


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
LOG_DEBUG = handler.get_subkey_value("controller", "LOG_DEBUG")
LOG_LEVEL = handler.get_subkey_value("controller", "LOG_LEVEL")


variables = Variables()

# Initialize the rclpy library
rclpy.init()

rclpy.logging.set_logger_level("controller", eval(LOG_LEVEL))

# Instance the serverTCP class
controller = Controller(TIMER_PERIOD, STEPWISE, LOG_DEBUG)

# Begin looping the node
rclpy.spin(controller)
    


    