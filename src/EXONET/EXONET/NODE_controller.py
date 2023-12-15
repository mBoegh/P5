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

    def __init__(self):
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

        self.upper_bound = 110-2 # deg
        self.lower_bound = 70+2 # deg

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

        self.pi = PID(50, 20, 0, setpoint=variables.t_pos)
        self.pi.output_limits = (-100, 100)
        
        # Initialising class Variables
        self.prev_duty_cycle = 0
        self.prev_vel = 0

        self.prev_spring_acc = 0
        self.prev_gravity_acc = 0

        self.lower_inertia = (1/3)*(variables.av_arm_weight+variables.exo_weight)*np.square(variables.l2_lenght) #kg*m2
        self.payload_inertia = variables.av_payload_weight*np.square(variables.l2_lenght) #kg*m2

        self.toggle_EEG_parameter = False

        self.called_manual_control_data_topic_callback = False
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



        # Initialising a publisher to the topic 'Motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int8 which is imported as Int8.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.target_velocity_publisher = self.create_publisher(Int16, 'Target_velocity', 10)
        self.target_velocity_publisher  # prevent unused variable warning



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

        self.get_logger().info(f"Recieved topic data: {msg.data}")

        self.called_eeg_data_topic_callback = True

        try:

            if self.toggle_EEG_parameter == True:
                data_string = msg.data

                if "," in data_string and "/" in data_string:
                    breaker_index = data_string.index("/")

                    data_string = data_string[:breaker_index]

                    seperator_index = data_string.index(",")

                    mental_command = data_string[:seperator_index]
                    
                    command_power = int(data_string[seperator_index+1:])

                if mental_command == "Lift":
                    variables.t_vel = self.map_range(command_power, 0, 100, 0, 40)
                    controller.pi.setpoint = variables.t_vel

                elif mental_command == "Drop":
                    variables.t_vel = self.map_range(-command_power, -100, 0, -40, 0)
                    controller.pi.setpoint = variables.t_vel

                elif mental_command == "Neutral":
                    variables.t_vel = 0
                    controller.pi.setpoint = variables.t_vel

                else:
                    self.get_logger().warning(f"Unexpected mental command in recieved EEG data.")

        except Exception as e:
            self.get_logger().warning(f"Unexpected EEG data with error: {e}")


    def manual_velocity_control_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'manual_control_subscription'
        """ 

        self.called_manual_control_data_topic_callback = True

        if self.toggle_EEG_parameter == False:
            variables.t_vel = msg.data # Target velocity
            self.get_logger().debug(f"Updated target joint velocity: {variables.t_vel}")

            controller.pi.setpoint = variables.t_vel


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
            """
            Stops the motor if a movement will exceed the angular limits of the exoskeleton
            """
            variables.t_vel = 0

            self.get_logger().debug(f"Physical joint limits exceed. Target velocity: {variables.t_vel}")
            self.pi.setpoint = variables.t_vel
            
            self.get_logger().debug(f"Updated controller setpoint: {self.pi.setpoint}")

            self.prev_duty_cycle = 0
            self.get_logger().debug(f"Reset previous dutycycle: {self.prev_duty_cycle}")

            self.duty_cycle_msg.data = 0
            self.velocity_motor_signals_publisher.publish(self.duty_cycle_msg)
            self.get_logger().debug(f"Published dutycycle: {self.duty_cycle_msg.data}")

        variables.elbow_joint_angle = msg.data

        self.get_logger().debug(f"Recieved data '{msg.data}'")
        self.get_logger().debug(f"Updated current joint angle with value: {variables.elbow_joint_angle}")

        # Limits the motion of the arm, by calling the edgeguard() function
        # if the arm angle becomes greater or smaller than the constants below
        if variables.elbow_joint_angle >= variables.upper_bound and variables.t_vel > 0:
            edge_guard()
        elif variables.elbow_joint_angle <= variables.lower_bound and variables.t_vel < 0: 
            edge_guard()
            

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

            if variables.t_vel == 0:
                self.pi.auto_mode = False
                self.pi.set_auto_mode(True, 0)

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
            
            spring_acc = spring_compensation_torque/(self.lower_inertia + self.payload_inertia) #((1.65 *gravity_compensation_torque + 10.24)/12)*100
            spring_vel = (spring_acc + self.prev_spring_acc) * 0.5  * (current_time - variables.prev_time)

            self.prev_spring_acc = spring_acc

            # Constants in below code found through testing the motor, and making a regression over the test data
            #spring_compensation_duty_cycle = 1.65 * spring_compensation_torque + 10.24 #((1.65 * spring_compensation_torque + 10.24)/12)*100 # PWM Duty cycle signal

            # Log the results from dynamic calculations for spring compensation
            self.get_logger().debug(
                f"Spring Compensation Calculations:"
               # f"\n- spring_arm_angle: {spring_arm_angle}"
               # f"\n- tense_spring_length: {tense_spring_length}"
               # f"\n- spring_angle: {spring_angle}"
               # f"\n- spring_force_vector: {spring_force_vector}"
               # f"\n- spring_tangential_force_component: {spring_tangential_force_component}"
               # f"\n- spring_compensation_torque: {spring_compensation_torque}"
               # f"\n- spring_gravity_compensation_duty_cycle: {spring_compensation_duty_cycle}"
            )

            # Dynamic calculations for gravity compensation
            fg2 = (variables.av_arm_weight + variables.exo_weight) * variables.g_acceleration
            fgp = variables.av_payload_weight * variables.g_acceleration

            f2 = np.cos(np.deg2rad((variables.elbow_joint_angle+90) - variables.shoulder_joint_angle)) * (variables.l2_lenght / 2) * fg2
            fp = np.cos(np.deg2rad((variables.elbow_joint_angle+90) - variables.shoulder_joint_angle)) * variables.l2_lenght * fgp

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
            cable_length = np.sqrt(np.square(variables.lm1_length)+np.square(variables.lm2_length)-2*variables.lm1_length*variables.lm2_length*np.cos(np.deg2rad(variables.elbow_joint_angle)))
            variables.cable_angle = np.rad2deg(np.arccos(np.divide((np.square(cable_length) + np.square(variables.lm2_length) - np.square(variables.lm1_length)), (2*cable_length*variables.lm2_length))))

            # Force cable components 1 and 2
            fc1 = torque_joint / variables.lm2_length
            fc2 = np.tan(np.deg2rad((np.pi / 2) - variables.cable_angle)) * fc1

            # Log force cable calculations
            self.get_logger().debug(
                f"Force Cable Calculations:"
                f"\n- fc1: {fc1}"
                f"\n- fc2: {fc2}"
            )

            # Find the cable force and calculate the motor torque
            fc = np.sqrt(fc1**2 + fc2**2)
            gravity_compensation_torque = fc * variables.spool_radius

            # Log the above calculated torque
            self.get_logger().debug(
                f"Torque Motor Calculation:"
                f"\n- Torque Motor: {gravity_compensation_torque}"
            )

            # Below code used to convert motor torque into volts, the numbers 3.301 and 10.24
            # was found by testing the motor, and then by making a regression over the resulting data 
            
            gravity_acc = gravity_compensation_torque/(self.lower_inertia + self.payload_inertia) #((1.65 *gravity_compensation_torque + 10.24)/12)*100
            gravity_vel = (gravity_acc + self.prev_gravity_acc) * 0.5  * (current_time - variables.prev_time)

            self.prev_gravity_acc = gravity_acc

            # Log duty cycle calculations
            self.get_logger().debug(
                f"Duty Cycle Calculations:"
                f"\n- Compensation Duty Cycle: {gravity_vel}"
            )

            # The controller
            angle_comp = 0
            if variables.t_vel < 0:
                angle_comp = (variables.elbow_joint_angle-45)/2
            elif variables.t_vel > 0:
                angle_comp = (180-variables.elbow_joint_angle-110)/4
            
            #- angle_comp
            variables.t_pos = variables.elbow_joint_angle+(variables.t_vel + (spring_vel + gravity_vel) + self.prev_vel - angle_comp) * 0.5  * (current_time - variables.prev_time)

            self.get_logger().info(f"Target Position: {variables.t_pos}")
            self.get_logger().info(f"Current Position: {variables.elbow_joint_angle}")
            controller.pi.setpoint = variables.t_pos
            duty_cycle = self.pi(variables.elbow_joint_angle)
            
            self.prev_vel = variables.t_vel
            
            if duty_cycle > 100:
                duty_cycle = 100
            if duty_cycle < -100:
                duty_cycle = -100

            # Log controller calculations
            self.get_logger().debug(
                f"Controller Calculations:"
                #f"\n- Control: {regulator}"
                f"\n- Duty Cycle: {duty_cycle}"
                #f"\n- Previous duty cycle: {self.prev_duty_cycle}"
                #f"\n- Compensation duty cycle: {(gravity_compensation_torque + spring_compensation_torque)*50} <-----" # + spring_compensation_duty_cycle
                # f"\n- Spri compensation duty cycle: {spring_compensation_torque} <-----"
            )

            self.prev_duty_cycle = duty_cycle
            variables.prev_time = current_time
            
            # Load msg with duty cycle data
            self.get_logger().info(f"Duty Cycle: {duty_cycle}")

            duty_cycle = int(duty_cycle)

            self.get_logger().debug(f"Duty cycle rounded: {duty_cycle}")

            self.duty_cycle_msg.data = duty_cycle

            if self.pi.setpoint == 0:
                self.duty_cycle_msg.data = 0

            self.get_logger().debug(f"Duty cycle message data: {self.duty_cycle_msg.data}")

            self.target_velocity_msg.data = variables.t_vel
            self.target_velocity_publisher.publish(self.target_velocity_msg)

            # Publish msg using velocity_motor_signals_publisher on topic 'Velocity_motor_signals'
            self.velocity_motor_signals_publisher.publish(self.duty_cycle_msg)

            # Log info
            self.get_logger().info(f"Published data: '{self.duty_cycle_msg.data}'")

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
    


    