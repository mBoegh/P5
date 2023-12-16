from EXONET.EXOLIB import JSON_Handler
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8, Int16, UInt16, Float32

from customtkinter import *
from customtkinter import StringVar, CTkSwitch 
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import matplotlib.dates as mdates
from datetime import datetime, timedelta


class Variables:
    """
    Class for initialising, setting and getting Variables.
    When instanced in global space, then every other scope can access the Variables.
    """

    def __init__(self):

        # Contains the mental command yielded from segmenting the recieved message on the topic 'EEG_data'
        self.mental_command = "EEG off"

        # Contains the eeg data yielded from segmenting the recieved message on the topic 'EEG_data'
        self.eeg_data = 0

        # Contains the current motor duty cycle
        self.duty_cycle = 0

        # Contains the exoskeletons current elbow joint angle
        self.current_angle = 90

        # Contains the exoskeletons current elbow joint velocity
        self.current_velocity = 0


class Gui(Node):
    """
    This is the GUI node of the EXONET ROS2 network.
    The purpose of the GUI is to make the prototype exoskeleton more developer friendly, 
    and to aid in debugging and testing.
    """

    def __init__(self, timer_period, upper_joint_angle_limit, lower_joint_angle_limit, increment_button_value, decrement_button_value, slider_zero, slider_lower_limit, slider_upper_limit, log_debug):

        # Initialising parsed Variables
        self.TIMER_PERIOD = timer_period
        self.UPPER_JOINT_ANGLE_LIMIT = upper_joint_angle_limit
        self.LOWER_JOINT_ANGLE_LIMIT = lower_joint_angle_limit
        self.INCREMENT_BUTTON_VALUE = increment_button_value
        self.DECREMENT_BUTTON_VALUE = decrement_button_value 
        self.SLIDER_ZERO = slider_zero
        self.SLIDER_LOWER_LIMIT = slider_lower_limit
        self.SLIDER_UPPER_LIMIT = slider_upper_limit
        self.LOG_DEBUG = log_debug

        # Initialising class Variables
        self.toggle_EEG_parameter = False  # Bool determining if the system reacts to the EEG datastream.

        # Initialising variable msg as being of data type 'std_msgs.msg.Int8' imported as Int8
        # The message is loaded with data and published to a topic
        self.msg = Int8()

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('gui')

        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

        # Initialising a subscriber to the topic 'EEG_data'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_subscription = self.create_subscription(String, 'EEG_data', self.eeg_data_topic_callback, 10)
        self.eeg_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Velocity_motor_signals'.
        # On this topic is expected data of type std_msgs.msg.Int16 which is imported as Int16.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.velocity_motor_signals_subscription = self.create_subscription(Int16, 'Velocity_motor_signals', self.motor_signals_topic_callback, 10)
        self.velocity_motor_signals_subscription  # prevent unused variable warning

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

        # Initialising a publisher to the topic 'EEG_toggle'.
        # On this topic is published data of type std_msgs.msg.Bool which is imported as Bool.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_toggle_publisher = self.create_publisher(Bool, 'EEG_toggle', 10)
        self.eeg_toggle_publisher  # prevent unused variable warning

        # Initialising a publisher to the topic 'Manual_position_control_data'.
        # On this topic is published data of type std_msgs.msg.UInt16 which is imported as UInt16.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_position_control_data_publisher = self.create_publisher(UInt16, 'Manual_position_control_data', 10)
        self.manual_position_control_data_publisher  # prevent unused variable warning

        # # Initialising a publisher to the topic 'Manual_input_velocity_control_data'.
        # # On this topic is published data of type std_msgs.msg.Int16 which is imported as Int16.  ############################################## Potentially deprecated. Needs test without
        # # The '10' argument is some Quality of Service parameter (QoS).
        # self.manual_input_velocity_control_data_publisher = self.create_publisher(Int16, 'Manual_input_velocity_control_data', 10)
        # self.manual_input_velocity_control_data_publisher  # prevent unused variable warning

        # Initialising a publisher to the topic 'Manual_velocity_control_data'.
        # On this topic is published data of type std_msgs.msg.Int16 which is imported as Int16.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_velocity_control_data_publisher = self.create_publisher(Int16, 'Manual_velocity_control_data', 10)
        self.manual_velocity_control_data_publisher  # prevent unused variable warning

        # Create a timer which periodically calls the specified callback function at a defined interval. ########################################### Potentially deprecated. Needs test without
        #self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

        # Instancing the main gui window. It has None parent and meaning that it does not belong to any other CTK object.
        # The ROS2 Humble logger is parsed so that it may be used in the instance, even tho the instance itself is not inheriting from the Node class.
        self.app = ParentWindow_MainMenu(None, self.get_logger())

        # Set the PWM / Duty Cycle indicator bar of the class Frame_InfoExo to 0.
        # This bar shows the current PWM / Duty Cycle visually in a *progress bar* esque manner.
        self.app.exo_frame.duty_cycle_bar.set(0)

        # Possibly redundant
        self.app.visual_frame.animate()     # Redraws the frame which contains the Exoskeleton visualization
        self.app.eeg_frame.animate()        # Redraws the frame which contains the EEG data visualization
        self.app.duty_cycle_frame.animate() # Redraws the frame which contains the PWM / duty cycle visualization
        self.app.feedback_frame.animate()   # Redraws the frame which contains the feedback (joint velocity and joint angle) visualization

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window.
        self.app.update_idletasks()
        self.app.update()


    def eeg_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'EEG_data' using the subscription 'eeg_data_subscription'.
        """

        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Received topic data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is formatted as a string f"{mental_command},{command_power}/" where comma is seperating the two values and forward slashs depicts the end of a value set.
        data_string = msg.data

        # try / except statement used for catching errors, logging them and continuing with program execution.
        try:

            # This is true if the toggle button labled 'EEG' in the main window is toggled True.
            if self.toggle_EEG_parameter == True:

                # This is true if both ',' and '/' are present in the string.
                # This is checked to make sure that the recieved data follows the formatting standard the developer chose in Node-Red.
                if "," in data_string and "/" in data_string:

                    # Find the index of the first forward slash in the string.
                    breaker_index = data_string.index("/")

                    # Reinitialize data_string with the string until and without the forward slash to seperate the first message from any other messages there might be in recieved.
                    data_string = data_string[:breaker_index]

                    # Find the index of the first comma in the string.
                    seperator_index = data_string.index(",")

                    # Reinitialize variable mental_command in the Variables class with the string data of index up until the comma disregarding the comma.
                    data.mental_command = data_string[:seperator_index]
                    
                    # Reinitialize variable command_power in the Variables class with the string data of index from from but without comma until the end of the string.
                    data.command_power = int(data_string[seperator_index+1:])

                    # Update the direction_label which shows the current mental command recieved, meaning the direction of movement.
                    self.app.exo_frame.direction_label.configure(text= data.mental_command)

                else:
                    # Log warning of unexpected message format.
                    self.get_logger().warning(f"Unexpected EEG data: {msg.data} \nIsolated message as: {data_string}")

        except Exception as e:
            # Log error message as error level of importance.
            self.get_logger().error(f"Unexpected EEG data with error: {e}")
        
            # Update the direction_label which shows the current mental command recieved, meaning the direction of movement, to display "Error".
            data.mental_command = "Error"
            self.app.exo_frame.direction_label.configure(text= data.mental_command)


        # The below functions are what actually does the updating of the window.
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window.
        self.app.update_idletasks()
        self.app.update()
        self.app.eeg_frame.animate()

    def motor_signals_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Velocity_motor_signals' using the subscription 'motor_signals_subscription'.
        """

        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is formatted as an integer from -100 to 100.
        data.duty_cycle = msg.data
        
        # This statement is true when the current duty cycle is greater than zero.
        # If true then the direction_label label in the Frame_InfoExo is updated with the direction of movement being "Lift".
        if data.duty_cycle > 0:
            # Update the direction_label which shows the current direction of movement to display "Lift".
            self.app.exo_frame.direction_label.configure(text= "Lift")
        
        # This statement is true when the current duty cycle is less than zero.
        # If true then the direction_label label in the Frame_InfoExo is updated with the direction of movement being "Drop".
        elif data.duty_cycle < 0:
            # Update the direction_label which shows the current direction of movement to display "Drop".
            self.app.exo_frame.direction_label.configure(text= "Drop")
        
        # If none of the above statements are true, then the direction_label label in the Frame_InfoExo is updated with the direction of movement being "Neutral".
        else:
            # Update the direction_label which shows the current direction of movement to display "Neutral".
            self.app.exo_frame.direction_label.configure(text= "Neutral")

        # Set the PWM / duty cycle bar to the current duty cycle. The PWM / duty cycle bar takes a value 0-1 so the duty cycle is made absolute and divided by 100.
        self.app.exo_frame.duty_cycle_bar.set(abs(data.duty_cycle) / 100)

        # Set the duty_cycle_data_label to display the current value duty cycle.
        self.app.exo_frame.duty_cycle_data_label.configure(text=data.duty_cycle) # Update the content of the CurrentAngle Label.

        # The below functions are what actually does the updating of the window.
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window.
        self.app.update_idletasks()
        self.app.update()
        self.app.duty_cycle_frame.animate()

    def feedback_joint_velocity_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Feedback_joint_velocity' using the subscription 'feedback_joint_velocity_subscription'.
        """

        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is formatted as a signed float value.
        data.current_velocity = msg.data

        # # This statement is true if the velocity_control_window is open.
        # # If that is the case then the current_velocity_label of the velocity_control_window is set to display the current velocity measured on the exoskeleton elbow joint.
        # if self.app.velocity_control_window is not None:
        #     self.app.velocity_control_window.current_velocity_label.configure(text= data.current_velocity) ################################################################## Potentially deprecated, needs test without
        
        # The current_velocity_label of the exoskeleton info frame in the main window is set to display the current velocity measured on the exoskeleton.
        self.app.exo_frame.current_velocity_data_label.configure(text= data.current_velocity)

        # The below functions are what actually does the updating of the window.
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window.
        self.app.update_idletasks()
        self.app.update()
        self.app.feedback_frame.animate()


    def feedback_joint_angle_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the topic 'Feedback_joint_angle' using the subscription 'feedback_joint_angle_subscription'.
        """
        
        # Log received message data as debug level of importance.
        self.get_logger().debug(f"Recieved topic data: '{msg.data}'")

        # Unpack the recieved message data.
        # The expected data is formatted as a signed float value.
        data.current_angle = msg.data

        # # This statement is true if the position_control_window is open.
        # # If that is the case then the current_angle_label of the position_control_window is set to display the current angle measured on the exoskeleton elbow joint.
        # if self.app.position_control_window is not None:
        #     self.app.position_control_window.current_angle_label.configure(text= data.current_angle) ##############################################################################3 Potentially deprecated, needs test without

        # The current_angle_label of the exoskeleton info frame in the main window is set to display the current angle measured on the exoskeleton
        self.app.exo_frame.current_angle_data_label.configure(text= data.current_angle)

        # The below functions are what actually does the updating of the window.
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window.
        self.app.update_idletasks()
        self.app.update()
        self.app.feedback_frame.animate()
        


    # def timer_callback(self):
    #     """
    #     Function called at specific time interval, specified in 'settings.json'.
    #     """
        
    #     # This is true if the toggle button labled 'EEG' in the main window is toggled True
    #     if self.toggle_EEG_parameter:

    #         # Load msg with current angle set in GUI 
    #         self.msg.data = data.current_angle

    #         # Publish msg using manual_control_data_publisher on topic 'Manual_control_data' ############################################### Potentially deprecated, needs test without
    #         self.manual_control_data_publisher.publish(self.msg)

    #         # Log info
    #         self.get_logger().debug(f"Published data: '{self.msg.data}'")


class ParentWindow_MainMenu(CTk):
    """
    Main window of the GUI. Instances the Child windows.
    """

    def __init__(self, parent, logger):
        super().__init__(parent)

        # Instance the ROS2 Humble logger from the parsed argument.
        self.logger = logger

        # Set window size in pixels.
        self.geometry("1200x800")
        
        # self.parent = parent ############################################################# Potentially deprecated. Needs test without

        # Set window title.
        self.title("P5 GUI - Main Menu")

        # Main function call.
        self.mainWidgets()

        # Initializing Variables used in keeping track of any open child windows.
        self.velocity_control_window = None
        self.position_control_window = None


    def mainWidgets(self):
        """
        Calls and arranges all frames needed in the main window.
        Also makes and positiones the buttons which create child windows.
        """

        # Initialize the information frame of the main window.
        self.exo_frame = Frame_InfoExo(self, self.logger)

        # Initialize the frame with all the buttons of the main window.
        self.manual_frame = Frame_MainMenu(self, self.logger)

        # Initialize exoskeleton visual representation plot.
        self.visual_frame = Frame_VisualCurrentExoAngle(self, self.logger)

        # Initialize the system data plots.
        self.eeg_frame = Frame_VisualEegData(self, nb_points= 100, logger= self.logger)
        self.duty_cycle_frame = Frame_VisualDutyCycle(self, nb_points= 100, logger= self.logger)
        self.feedback_frame = Frame_VisualJointFeedback(self, nb_points= 100, logger= self.logger)

        # Poistion all the frames in the main windows grid.
        self.exo_frame.grid(row= 0, column= 0, pady= 20, padx= 60)
        self.manual_frame.grid(row= 1, column= 0, pady= 20, padx= 60)
        self.visual_frame.grid(row= 2, column= 0, pady= 10, padx= 5)
        self.eeg_frame.grid(row= 2, column= 1, pady=10, padx= 5)
        self.duty_cycle_frame.grid(row= 3, column= 0, pady=10, padx= 5)
        self.feedback_frame.grid(row= 3, column= 1, pady=10, padx= 5)

        # Create the buttons of the Frame_MainMenu and positions them in the frame. 
        # These do not work when created in that class, but developers know that that would make more sense to do.
        self.position_control_button = CTkButton(master=self.manual_frame, text="Position Control", command=self.open_position_control_menu)
        self.position_control_button.grid(row= 1, column= 1, padx= 10, pady= 5)
        self.velocity_control_button = CTkButton(master=self.manual_frame, text="Velocity Control", command=self.open_velocity_control_menu)
        self.velocity_control_button.grid(row= 1, column= 2, padx= 10, pady= 5)


    def open_velocity_control_menu(self):
        """
        This function first checks if the position_control_window is open and destroys it if it is.
        Then it check if the velocity_control_window is already open.
        If the window is open, then it is focused and lifted to the front. Otherwise the window is opened.
        """

        # This statement is true if the position_control_window is open.
        # If true, then the position_control_window is closed.
        if gui.app.position_control_window:
            gui.app.position_control_window.destroy()

        # This statement is true if the velocity_control_window is not open.
        # If true then the velocity_control_window is opened and the ROS2 Humble logging system is parsed to the class, so it may be used in that scope.
        if self.velocity_control_window is None or not self.velocity_control_window.winfo_exists():
            self.velocity_control_window = ChildWindow_VelocityControl(self.logger)
        
        # If false then the velocity_control_window is focused and lifted to the front.
        else:
            self.velocity_control_window.focus()
            self.velocity_control_window.lift()


    def open_position_control_menu(self):
        """
        This function first checks if the velocity_control_window is open and destroys it if it is.
        Then it check if the position_control_window is already open.
        If the window is open, then it is focused and lifted to the front. Otherwise the window is opened.
        """
        
        # This statement is true if the velocity_control_window is open.
        # If true, then the velocity_control_window is closed.
        if gui.app.velocity_control_window: 
            gui.app.velocity_control_window.destroy()
    
        # This statement is true if the position_control_window is not open.
        # If true then the position_control_window is opened and the ROS2 Humble logging system is parsed to the class, so it may be used in that scope.
        if self.position_control_window is None or not self.position_control_window.winfo_exists():
            self.position_control_window = ChildWindow_PositionControl(self.logger)

        # If false then the position_control_window is focused and lifted to the front.
        else:
            self.position_control_window.focus()
            self.position_control_window.lift()


class ChildWindow_VelocityControl(CTkToplevel):
    """
    This window contains everything related to manual velocity control.
    """

    def __init__(self, logger):

        # Initialize the parent class which this class is inheriting from.
        CTkToplevel.__init__(self)

        # Set window size in pixels.
        self.geometry("400x300")

        # Set window title.
        self.title("P5 GUI - Velocity Control")

        # Instance the ROS2 Humble logger from the parsed argument.
        self.logger = logger

        # Instance the target velocity.
        self.target_velocity = 0

        # Instance message Variables as Int16 datatype from ROS2 Humble.
        self.velocity_control_msg = Int16()
       
       # self.input_velocity_control_msg = Int16() ######################################################### potentially deprecated, need test without

        # Create the 'Exit' button for this window, and place it in the window.
        # This button closes the window.
        self.exit_button = CTkButton(self, text= "Exit", command= self.exit_button_event)
        self.exit_button.grid(row=1, column=0, padx=10, pady=5)

        # Create the 'Stop' button, and place it in the window.
        # This button sets target velocity to zero and resets the slider to zero.
        self.manual_stop_button = CTkButton(self, text= "Stop", command= self.manual_stop_event)
        self.manual_stop_button.grid(row=2, column=0, padx=10, pady=5)

        # Create the slider for velocity control, and place it in the window.
        # The slider calls the slider_event function every time it is pressed / changed, with the sliders current value as argument.
        # The slider outputs a value in deg/s, such that the lower limit is -40 deg/s and upper is 40 deg/s.
        self.slider = CTkSlider(self, from_= gui.SLIDER_LOWER_LIMIT, to= gui.SLIDER_UPPER_LIMIT, command= self.slider_event, width= 240, number_of_steps=80)
        self.slider.grid(row=2, column=1, padx=10, pady=5, columnspan=2)

        # Create the target_velocity_data_label, and place it in the window.
        # This label shows the computed current velocity in the exoskeleton elbow joint.
        self.target_velocity_label = CTkLabel(self, text= f"Target velocity:    {str(self.target_velocity)}")
        self.target_velocity_label.grid(row= 3, column= 1, padx= 10, pady= 5)

        # Create the entry field, and place it in the window.
        # The entry field is a user input field where the user can input a value target velcoity in deg/s.
        self.entry = CTkEntry(self,
            placeholder_text="Deg/sec",
            height=50,
            width=200,
            font=("Helvetica", 18),
            corner_radius=10,
            text_color="black",
            placeholder_text_color="grey",
            fg_color=("system", "white"),  # outer, inner
            state="normal",
        )
        self.entry.grid(row=4, column=1, padx=10, pady=5)

        # Create the 'Submit' button, and place it in the window.
        # This button submits the current value in the entry field as the systems new target velocity.
        submit_button = CTkButton(self, text="Submit", command= self.submit)
        submit_button.grid(row=5, column=1, padx=10, pady=5)

        # Bind the 'Return' / 'Enter' key to the submit method, so that one can press 'Return' on the keyboard instead of clicking the button manually.
        self.entry.bind("<Return>", lambda event: self.submit())


    def exit_button_event(self):
        """
        Destroy the window, ie. close the window.
        """

        # Closes the window.
        self.destroy()


    def slider_event(self, value):
        """
        Function called whenever the slider is manipulated.
        Publishes slider value to topic
        'Manual_velocity_control_data'.
        """

        # Loads the velocity_control_msg with the current value in the entry field.
        self.velocity_control_msg.data = int(value)

        # Set the target_velocity_label to display the slider value.
        self.target_velocity_label.configure(text= f"Target velocity:    {str(self.velocity_control_msg.data)}")

        # Publishes the velocity_control_msg using the manual_velocity_control_data_publisher to the topic /Manual_velocity_control_data.
        gui.manual_velocity_control_data_publisher.publish(self.velocity_control_msg)

        # Logs the published target velocity as Debug level of importance.
        self.logger.debug(f"Target velocity: {value}")


    def manual_stop_event(self):
        """
        Function called whenever the Stop button is pressed.
        Sets the slider to 0, thereby stopping the motor.
        """

        # Resets the slider to the value of gui.SLIDER_ZERO, which is set in settings.json.
        self.slider.set(gui.SLIDER_ZERO)

        # Publishes the value of gui.SLIDER_ZERO as new target velocity, which is set in settings.json.
        self.slider_event(gui.SLIDER_ZERO)

        # Logging that the event is triggered with Info level of importance.
        self.logger.debug("User triggered stop")


    def clear(self):
        """
        Clears the entry field of any characters.
        """

        # Removes all characters currently in the entry field.
        self.entry.delete(0, END)


    def submit(self):
        """
        Publishes integer values in the entry field.
        Can be triggered with 'RETURN' button.
        """

        # Try/except statement for catching any errors. Used as a precaution to except entry field values not suited for Int16 data type.
        try:

            # Loads the velocity_control_msg with the current value target velocity in the entry field.
            self.velocity_control_msg.data = int(self.entry.get())

            # Publishes the target velocity to the topic /Manual_velocity_control_data.
            gui.manual_velocity_control_data_publisher.publish(self.velocity_control_msg)

            # Logs the published data with Debug level of importance.
            self.logger.debug(f"Published data: '{self.velocity_control_msg.data}'")

            # Sets the slider to match the value input in the entry field.
            self.slider.set(self.velocity_control_msg.data)

            # Set the target_velocity_label to display the submitted value.
            self.target_velocity_label.configure(text= f"Target velocity:    {str(self.velocity_control_msg.data)}")
        
        except Exception as e:

            # Log that the publish attempt failed, the data attempted to publish and the error message the system excepted.
            self.logger.warning(f"Failed to publish data: '{self.velocity_control_msg.data}' With error: {e}")

        # Clear the entry field so it is ready for a new entry from the user.
        self.clear()


class ChildWindow_PositionControl(CTkToplevel):
    """
    This window contains everything related to manual position control.
    """

    def __init__(self, logger):

        # Initialize the parent class which this class is inheriting from.
        CTkToplevel.__init__(self)

        # Set window size in pixels
        self.geometry("400x300")

        # Set window title
        self.title("P5 GUI - Position Control")

        # Initialize the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # Initialize message variable as UInt16 datatype from ROS2 Humble
        self.position_control_msg = UInt16()

        # Initialize the target_angle variable as the value current_angle is initialized as in the Variables class
        self.target_angle = data.current_angle
        
        # Create the target_angle_label, and place it in the window.
        # This label shows the computed current velocity in the exoskeleton elbow joint.
        self.target_angle_label = CTkLabel(self, text= f"Target angle:    {str(self.target_angle)}")
        self.target_angle_label.grid(row= 3, column= 1, padx= 10, pady= 5)

        # Create the 'v' button, and place it in the window.
        # This button decrements the current target joint angle position with one degree.
        self.manual_down_button = CTkButton(self, text="v", command= self.manual_down_event)
        self.manual_down_button.grid(row= 2, column= 2, padx= 10, pady= 5)
        
        # Create the '^' button, and place it in the window.
        # This button increments the current target joint angle position with one degree.
        self.manual_up_button = CTkButton(self, text="^", command= self.manual_up_event)
        self.manual_up_button.grid(row= 2, column= 0, padx= 10, pady= 5)

        # Create the 'Exit' button for this window, and place it in the window
        # This button closes the window.
        self.exit_button = CTkButton(self, text="Exit", command= self.exit_button_event)
        self.exit_button.grid(row= 0, column= 0, padx= 10, pady= 5)

        # Create the entry field, and place it in the window.
        # The entry field is a user input field where the user can input a value target position in deg.
        self.entry = CTkEntry(self,
            placeholder_text="deg",
            height=50,
            width=200,
            font=("Helvetica", 18), # Font, font size
            corner_radius=10,
            text_color="black",
            placeholder_text_color="grey",
            fg_color=("system", "white"),  # outer, inner
            state="normal",
        )
        self.entry.grid(row=2, column=1, padx=10, pady=5)

        # Create the 'Submit' button, and place it in the window.
        # This button submits the current value in the entry field as the systems new target position.
        submit_button = CTkButton(self, text="Submit", command= self.submit)
        submit_button.grid(row=1, column=1, padx=10, pady=5)

        # Bind the 'Return' / 'Enter' key to the submit method, so that one can press 'Return' on the keyboard instead of clicking the button manually.
        self.entry.bind("<Return>", lambda event: self.submit())


    # Define Functions used in the Manual Control frame
    def manual_up_event(self):
        """
        Function called when the '^' button is pressed.
        Increases the target joint angle with the amount of degrees specified in INCREMENT_BUTTON_VALUE in settings.json.
        """
        
        # Reinstance the target_angle variable as the current exoskeleton elbow joint angle summed with the INCREMENT_BUTTON_VALUE value specified in settings.json.
        self.target_angle = self.target_angle + gui.INCREMENT_BUTTON_VALUE

        # This statement is true if the UPPER_JOINT_ANGLE_LIMIT is exceeded. This value is set in settings.json.
        # If true then reinstance target_angle as the value of UPPER_JOINT_ANGLE_LIMIT,
        # so that the system attempts to position itself on the limit.
        if data.current_angle > gui.UPPER_JOINT_ANGLE_LIMIT or self.target_angle > gui.UPPER_JOINT_ANGLE_LIMIT: 
            self.target_angle = gui.UPPER_JOINT_ANGLE_LIMIT

        # Loads the position_control_msg with the target_angle.
        self.position_control_msg.data = self.target_angle

        # Publishes the target angle position to the topic /Manual_position_control_data
        gui.manual_position_control_data_publisher.publish(self.position_control_msg)

        # Logs the published data with Debug level of importance
        self.logger.debug(f"Published data: '{self.position_control_msg.data}'")

        # Set the target_angle_label to display the submitted value.
        self.target_angle_label.configure(text= f"Target angle:    {str(self.position_control_msg.data)}")


    def manual_down_event(self):
        """
        Function called when the 'v' button is pressed.
        Decreases the target joint angle with the amount of degrees specified in INCREMENT_BUTTON_VALUE in settings.json.
        """
        
        # Reinstance the target_angle variable as the current exoskeleton elbow joint angle summed with the DECREMENT_BUTTON_VALUE value specified in settings.json.
        self.target_angle = self.target_angle - gui.DECREMENT_BUTTON_VALUE

        # This statement is true if the LOWER_JOINT_ANGLE_LIMIT is exceeded. This value is set in settings.json.
        # If true then reinstance target_angle as the value of LOWER_JOINT_ANGLE_LIMIT,
        # so that the system attempts to position itself on the limit.
        if data.current_angle < gui.LOWER_JOINT_ANGLE_LIMIT or self.target_angle < gui.LOWER_JOINT_ANGLE_LIMIT: 
            self.target_angle = gui.LOWER_JOINT_ANGLE_LIMIT

        # Loads the position_control_msg with the target_angle.
        self.position_control_msg.data = self.target_angle

        # Publishes the target angle position to the topic /Manual_position_control_data
        gui.manual_position_control_data_publisher.publish(self.position_control_msg)
        
        # Logs the published data with Debug level of importance
        self.logger.debug(f"Published data: '{self.position_control_msg.data}'")

        # Set the target_angle_label to display the submitted value.
        self.target_angle_label.configure(text= f"Target angle:    {str(self.position_control_msg.data)}")


    def exit_button_event(self): 
        """
        Destroy the window, ie. close the window.
        """

        # Closes the window.
        self.destroy()


    def clear(self):
        """
        Clears the entry field of any characters.
        """

        # Removes all characters currently in the entry field.
        self.entry.delete(0, END)


    def submit(self):
        """
        Publishes integer values in the entry field.
        Can be triggered with 'RETURN' button.
        """

        # Try/except statement for catching any errors. Used as a precaution to except entry field values not suited for UInt16 data type.
        try:

            # Gets the current value in the entry field and makes sure that the value is suitable for data type UInt16.
            value = abs(int(self.entry.get()))

            # This statement is true if the value inputted in the entry field is less than or equal to the LOWER_JOINT_ANGLE_LIMIT set in settings.json.
            # If true, then the target_angle is instanced as the LOWER_JOINT_ANGLE_LIMIT value.
            if (value <= gui.LOWER_JOINT_ANGLE_LIMIT): 
                self.target_angle = gui.LOWER_JOINT_ANGLE_LIMIT

            # This statement is true if the value inputted in the entry field is greater than or equal to the UPPER_JOINT_ANGLE_LIMIT set in settings.json.
            # If true, then the target_angle is instanced as the UPPER_JOINT_ANGLE_LIMIT value.
            elif (value >= gui.UPPER_JOINT_ANGLE_LIMIT):
                self.target_angle = gui.UPPER_JOINT_ANGLE_LIMIT
        
            # If none of the above statements are true, then the target_angle is instanced as the value inputted in the entry field.
            else:
                self.target_angle = value

            # Loads the position_control_msg with the current value target angle position in the entry field.
            self.position_control_msg.data = self.target_angle
        
            # Publishes the target angle position to the topic /Manual_position_control_data.
            gui.manual_position_control_data_publisher.publish(self.position_control_msg)

            # Set the target_angle_label to display the submitted value.
            self.target_angle_label.configure(text= f"Target angle:    {str(self.position_control_msg.data)}")

            # Logs the published data with Debug level of importance.
            self.logger.debug(f"Published data: '{self.position_control_msg.data}'")

        except Exception as e:

            # Log that the publish attempt failed, the data attempted to publish and the error message the system excepted.
            self.logger.warning(f"Failed to publish data: '{self.position_control_msg.data}' With error: {e}")

        # Clear the entry field so it is ready for a new entry from the user.
        self.clear()


class Frame_MainMenu(CTkFrame):
    """
    Frame wherein the main menu controls are placed.
    """

    def __init__(self, parent, logger):

        # Initialize the parent class which this class is inheriting from.
        super().__init__(parent)

        # Instance the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # Instance message variable as Bool datatype from ROS2 Humble
        self.eeg_toggle_msg = Bool()

        # self.parent = parent ############################################################# Potentially deprecated. Needs test without
        
        # Create the 'EEG' switch, and place it in the window.
        # The switch toggles the switch_var variable between True and False, instanced as False.
        # When the switch is toggled True for the first time, then the system attempts to connect with the EEG Node-Red system on TCP.
        # After this, then the switch toggles whether or not the system reacts to the EEG data stream.
        self.switch_var = StringVar(value="False")
        self.switch = CTkSwitch(self, text="EEG", command= self.eeg_toggle, variable= self.switch_var, onvalue= "True", offvalue= "False")
        self.switch.grid(row=1, column= 0, padx= 10, pady= 5)

    def eeg_toggle(self):
        """
        Function called when the EEG switch is toggled.
        The switch state determines wether or not the system shall attempt
        to connect with the EEG Node-Red system on TCP.
        When a connection is established, then this determines wether or not
        the system reacts to the EEG data stream.
        """

        # Instance value variable with the current value of the switch_var variable using the get() method.
        value = gui.app.manual_frame.switch_var.get()

        # This statement is true when the switch is toggled True.
        # If true then disable the child window buttons and close any child windows, so that no other control method can be used when the system is reacting to the EEG data stream.
        if value == "True":

            # Disable the child window buttons, so that no other control method can interfere when the system is reacting to the EEG data stream.
            gui.app.position_control_button.configure(state= DISABLED)
            gui.app.velocity_control_button.configure(state= DISABLED)
            
            # This statement is true when the position_control_window is open.
            # If true then the position_control_window is closed.
            if gui.app.position_control_window:
                gui.app.position_control_window.destroy()

            # This statement is true when the velocity_control_window is open.
            # If true then the velocity_control_window is closed.
            if gui.app.velocity_control_window: 
                gui.app.velocity_control_window.destroy()
   
            # Loads the eeg_toggle_msg with bool value True
            self.eeg_toggle_msg.data = True
            
            # Sets the toggle_EEG_parameter to True, so that any other dependent method knows that the system is reacting to the EEG data stream.
            gui.toggle_EEG_parameter == True

        else:

            # Enable the child window buttons, so that manual control methods can be used, since the system is not reacting to the EEG data stream.
            gui.app.position_control_button.configure(state= NORMAL)
            gui.app.velocity_control_button.configure(state= NORMAL)
            
            # Loads the eeg_toggle_msg with bool value False
            self.eeg_toggle_msg.data = False
            
            # Sets the toggle_EEG_parameter to False, so that any other dependent method knows that the system is not reacting to the EEG data stream.
            gui.toggle_EEG_parameter == False

        # Publishes the eeg toggle value to the topic /EEG_toggle.
        gui.eeg_toggle_publisher.publish(self.eeg_toggle_msg)

        # Logs the value of the switch with Info level of importance
        self.logger.debug(f"EEG switched {value}")


class Frame_InfoExo(CTkFrame):
    """
    Frame wherein information about the current state of the exoskeleton is shown.
    """

    def __init__(self, parent, logger):

        # Initialize the parent class which this class is inheriting from.
        CTkFrame.__init__(self, parent)

        # Instance the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # self.parent = parent ############################################################# Potentially deprecated. Needs test without
        
        # Create the duty_cycle_label label, and place it in the window.
        # This label shows a static text giving context for the duty_cycle_data_label label.
        self.duty_cycle_label = CTkLabel(self, text="PWM: ")
        self.duty_cycle_label.grid(row= 0, column= 0, padx= 10, pady= 5)

        # Create the duty_cycle_bar bar, and place it in the window.
        # This bar visually shows the power of the current duty cycle. Takes a value 0-1. 
        # Therefore the duty cycle is absolute without any context of direction being written and the direction context is given in the direction_label label.
        self.duty_cycle_bar = CTkProgressBar(self, orientation= "horizontal")
        self.duty_cycle_bar.grid(row= 0, column= 1, padx= 10, pady= 5)

        # Create the duty_cycle_data_label label, and place it in the window.
        # This label shows the current duty cycle sent to the motor.
        self.duty_cycle_data_label = CTkLabel(self, text= str(data.duty_cycle))
        self.duty_cycle_data_label.grid(row= 0, column=2, padx= 10, pady= 5)

        # Create the direction_label label, and place it in the window.
        # This label shows the current direction of movement for the exoskeleton:
        #  - "Neutral" being no movement
        #  - "Lift" being moving upwards
        #  - "Drop" being moving downwards
        self.direction_label = CTkLabel(self, text= "Neutral")
        self.direction_label.grid(row= 0, column= 3, padx= 10, pady= 5)

        # Create the current_velocity_label label, and place it in the window.
        # This label shows a static text giving context for the current_velocity_data_label label.
        self.current_velocity_label = CTkLabel(self, text= "Velocity: ")
        self.current_velocity_label.grid(row= 1, column= 0, padx= 10, pady= 5)

        # Create the current_velocity_data_label label, and place it in the window.
        # This label shows the computed current velocity of the exoskeleton elbow joint.
        self.current_velocity_data_label = CTkLabel(self, text= str(data.current_velocity))
        self.current_velocity_data_label.grid(row= 1, column= 1, padx= 10, pady= 5)

        # Create the current_angle_label label, and place it in the window.
        # This label shows a static text giving context for the current_angle_data_label label.
        self.current_angle_label = CTkLabel(self, text= "Angle: ")
        self.current_angle_label.grid(row= 2, column= 0, padx= 10, pady= 5)

        # Create the current_angle_data_label label, and place it in the window.
        # This label shows the computed current angle of the exoskeleton elbow joint.
        self.current_angle_data_label = CTkLabel(self, text= str(data.current_angle))
        self.current_angle_data_label.grid(row= 2, column= 1, padx= 10, pady= 5)
        

class Frame_VisualEegData(CTkFrame):
    """
    Makes and displays the graph for the EEG data, the class will need to be given how many data mounts
    which should be displayed on the graph.
    """

    def __init__(self, parent, nb_points, logger):
        
        # Initialize the parent class which this class is inheriting from.
        super().__init__(parent)

        # Instance the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # self.parent = parent ############################################################# Potentially deprecated. Needs test without

        # Define the graph, and configure the axes
        self.figure, self.ax = plt.subplots(figsize=(5,3), dpi=50)

        # Format the x-axis to show the current time of the computer, formatted as 'HOUR:MINUTE:SECOND'.
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

        # Initial x and y data
        date_time_obj = datetime.now() + timedelta(seconds=-nb_points)
        self.x_data = [date_time_obj + timedelta(seconds=i) for i in range(nb_points)]
        self.y_data = [0 for i in range(nb_points)]

        # Create the first plot
        self.plot = self.ax.plot(self.x_data, self.y_data, label='EEG data')[0]

        # Set axis limits
        self.ax.set_ylim(-100,100)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])
        
        # Draw a red, dashed line at zero
        self.ax.axhline(0, color='r', linestyle='--', linewidth=0.1)

        FrameTopLabel = CTkLabel(self, text="EEG data")
        FrameTopLabel.pack(pady=10, padx=10, side='top')
        self.canvas = FigureCanvasTkAgg(self.figure, self)
        self.canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=True)

    def animate(self):
        """
        Updates the Frame_VisualEegData matplotlib plot with new data.
        """

        # Append new data point to x and y data
        self.x_data.append(datetime.now())
        self.y_data.append(int(data.eeg_data))
        
        # Remove oldest datapoints
        self.x_data = self.x_data[1:]
        self.y_data = self.y_data[1:]
        
        # Update plot with new data
        self.plot.set_xdata(self.x_data)
        self.plot.set_ydata(self.y_data)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])

        # Draw a red, dashed line at zero
        self.ax.axhline(0, color='r', linestyle='--', linewidth=0.1)
        
        # Redraw plot
        self.canvas.draw_idle() 


class Frame_VisualDutyCycle(CTkFrame):
    """
    Makes and displays the graph for the EEG data, the class will need to be given how many data mounts
    which should be displayed on the graph.
    """

    def __init__(self, parent, nb_points, logger):
        
        # Initialize the parent class which this class is inheriting from.
        super().__init__(parent)

        # Instance the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # self.parent = parent ############################################################# Potentially deprecated. Needs test without

        # Define the graph, and configure the axes
        self.figure, self.ax = plt.subplots(figsize=(5,3), dpi=50)
        
        # Format the x-axis to show the current time of the computer, formatted as 'HOUR:MINUTE:SECOND'.
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

        # Initial x and y data
        date_time_obj = datetime.now() + timedelta(seconds=-nb_points)
        self.x_data = [date_time_obj + timedelta(seconds=i) for i in range(nb_points)]
        self.y_data = [0 for i in range(nb_points)]
        
        # Create the first plot
        self.plot = self.ax.plot(self.x_data, self.y_data, label='Duty cycle')[0]

        # Set axis limits
        self.ax.set_ylim(-100,100)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])
        
        # Draw a red, dashed line at zero
        self.ax.axhline(0, color='r', linestyle='--', linewidth=0.1)

        FrameTopLabel = CTkLabel(self, text="Duty cycle")
        FrameTopLabel.pack(pady=10, padx=10, side='top')
        self.canvas = FigureCanvasTkAgg(self.figure, self)
        self.canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=True)

    def animate(self):
        """
        Updates the Frame_VisualDutyCycle matplotlib plot with new data.
        """

        # Append new data point to x and y data
        self.x_data.append(datetime.now())
        self.y_data.append(int(data.duty_cycle))
        
        # Remove oldest datapoint
        self.x_data = self.x_data[1:]
        self.y_data = self.y_data[1:]
        
        # Update plot data with new data
        self.plot.set_xdata(self.x_data)
        self.plot.set_ydata(self.y_data)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])

        # Draw a red, dashed line at zero
        self.ax.axhline(0, color='r', linestyle='--', linewidth=0.1)
        
        # Redraw plot
        self.canvas.draw_idle()


class Frame_VisualJointFeedback(CTkFrame):
    """
    Makes and displays the graph for the EEG data, the class will need to be given how many data mounts
    which should be displayed on the graph.
    """

    def __init__(self, parent, nb_points, logger):
        
        # Initialize the parent class which this class is inheriting from.
        super().__init__(parent)

        # Instance the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # self.parent = parent ############################################################# Potentially deprecated. Needs test without

        # Define the graph, and configure the axes
        self.figure, self.ax = plt.subplots(figsize=(5,3), dpi=50)
        
        # Format the x-axis to show the current time of the computer, formatted as 'HOUR:MINUTE:SECOND'.
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

        # Initial x data
        date_time_obj = datetime.now() + timedelta(seconds=-nb_points)
        self.x_data = [date_time_obj + timedelta(seconds=i) for i in range(nb_points)]

        # Initial y data for two datasets
        self.y_data1 = [0 for _ in range(nb_points)]
        self.y_data2 = [0 for _ in range(nb_points)]

        # Create the first plot
        self.plot1, = self.ax.plot(self.x_data, self.y_data1, label='Velocity [deg/sec]')
        self.plot2, = self.ax.plot(self.x_data, self.y_data2, label='Angle [deg]')

        # Set axis limits
        self.ax.set_ylim(-50, 120)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])
        
        # Draw a red, dashed line at zero
        self.ax.axhline(0, color='r', linestyle='--', linewidth=0.1)

        FrameTopLabel = CTkLabel(self, text="Feedback")
        FrameTopLabel.pack(pady=10, padx=10, side='top')
        self.canvas = FigureCanvasTkAgg(self.figure, self)
        self.canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=True)

    def animate(self):
        """
        Updates the Frame_VisualJointFeedback matplotlib plot with new data.
        """

        # Append new data point to x and y data
        self.x_data.append(datetime.now())
        self.y_data1.append(int(data.current_velocity))
        self.y_data2.append(int(data.current_angle))
        
        # Remove oldest datapoints
        self.x_data = self.x_data[1:]
        self.y_data1 = self.y_data1[1:]
        self.y_data2 = self.y_data2[1:]
        
        # Update plot with new data
        self.plot1.set_xdata(self.x_data)
        self.plot1.set_ydata(self.y_data1)
        self.plot2.set_xdata(self.x_data)
        self.plot2.set_ydata(self.y_data2)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])

        # Draw a red, dashed line at zero
        self.ax.axhline(0, color='r', linestyle='--', linewidth=0.1)
        
        # Redraw plot
        self.canvas.draw_idle()



class Frame_VisualCurrentExoAngle(CTkFrame):
    """
    Frame wherein the current angle of the exoskeleton is illustrated.
    """

    def __init__(self, parent, logger):
        
        # Initialize the parent class which this class is inheriting from.
        super().__init__(parent)

        # Instance the ROS2 Humble logger from the parsed argument
        self.logger = logger

        # Length of the lines displayed in Frame_VisualCurrentExoAngle class
        self.length = 4

        # self.parent = parent ############################################################# Potentially deprecated. Needs test without
        
        # Calculate the end point for the movable arm
        endx = 2 + self.length * math.cos(math.radians((data.current_angle-90)))
        endy = 5 + self.length * math.sin(math.radians(data.current_angle-90))

        # Create the figure without content
        self.figure, self.ax = plt.subplots(figsize=(3,3), dpi=50) 

        # Set axis limits for the axis in the plot
        self.ax.set_ylim(0,10)
        self.ax.set_xlim(0,10)

        # Draw the plot in the figure
        self.grap = self.ax.plot([2,2,endx], [9,5,endy], 'bo-')

        # Sets the figure to be a canvas, such it can be drawn by customtkinter
        self.canvas = FigureCanvasTkAgg(self.figure, self)

        # Place the canvas in the frame
        self.canvas.get_tk_widget().pack(side='top', fill=BOTH, expand=True)

    
    def animate(self):
        """
        Used to redraw the plot. Needs to recalculate the end points for the movable arm.
        """

        # Recalculate the end point for the movable arm
        endx = 2 + self.length * math.cos(math.radians((data.current_angle-90)))
        endy = 5 + self.length * math.sin(math.radians(data.current_angle-90))

        # Clears all content on the plot, without removing the axes
        self.ax.cla()

        # Redefine the limits of the plot
        self.ax.set_ylim(0,10) 
        self.ax.set_xlim(0,10)

        # Redraw the exoskeleton visualization
        self.grap = self.ax.plot([2,2,endx], [9,5,endy], 'bo-') 
        
        # Redraw the canvas
        self.canvas.draw_idle() 


####################
####    MAIN    ####
####################

# Path for 'settings.json' file.
json_file_path = ".//src//EXONET//EXONET//settings.json"

# Instance the 'JSON_Handler' class for interacting with the 'settings.json' file.
handler = JSON_Handler(json_file_path)

# Get settings from 'settings.json' file.
TIMER_PERIOD = handler.get_subkey_value("gui", "TIMER_PERIOD")
UPPER_JOINT_ANGLE_LIMIT = handler.get_subkey_value("gui", "UPPER_JOINT_ANGLE_LIMIT")
LOWER_JOINT_ANGLE_LIMIT = handler.get_subkey_value("gui", "LOWER_JOINT_ANGLE_LIMIT")
INCREMENT_BUTTON_VALUE = handler.get_subkey_value("gui", "INCREMENT_BUTTON_VALUE")
DECREMENT_BUTTON_VALUE = handler.get_subkey_value("gui", "DECREMENT_BUTTON_VALUE")
SLIDER_ZERO = handler.get_subkey_value("gui", "SLIDER_ZERO")
SLIDER_UPPER_LIMIT = handler.get_subkey_value("gui", "SLIDER_UPPER_LIMIT")
SLIDER_LOWER_LIMIT = handler.get_subkey_value("gui", "SLIDER_LOWER_LIMIT")
LOG_DEBUG = handler.get_subkey_value("gui", "LOG_DEBUG")
LOG_LEVEL = handler.get_subkey_value("gui", "LOG_LEVEL")

# Change appearance of the GUI.
set_appearance_mode('system')
set_default_color_theme("blue")

# Instance the 'Variables' class in global scope, so every subscope can see, get and set variables.
data = Variables()

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
rclpy.logging.set_logger_level("gui", eval(LOG_LEVEL))

# Instance the node class.
gui = Gui(TIMER_PERIOD, UPPER_JOINT_ANGLE_LIMIT, LOWER_JOINT_ANGLE_LIMIT, INCREMENT_BUTTON_VALUE, DECREMENT_BUTTON_VALUE, SLIDER_ZERO, SLIDER_LOWER_LIMIT, SLIDER_UPPER_LIMIT, LOG_DEBUG)

# Counter for updating the graphs one at a time. 
# Increments by one. When the counter reaches four, it is reset to zero.
graph_update_count = 0

# Main loop.
while True:

    # Run the node once.
    rclpy.spin_once(gui, timeout_sec=0.01) # We spin once as to not get stuck

    # This statement is true when the graph_update_count counter variable is zero.
    # If true then update the visual representation of the exoskeleton and increase the counter by one.
    if graph_update_count == 0:
        gui.app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
        graph_update_count += 1 

    # This statement is true when the graph_update_count counter variable is one.
    # If true then update the EEG data graph and increase the counter by one.
    elif graph_update_count == 1:
        gui.app.eeg_frame.animate()
        graph_update_count += 1 

    # This statement is true when the graph_update_count counter variable is two.
    # If true then update the duty cycle graph and increase the counter by one.
    elif graph_update_count == 2:    
        gui.app.duty_cycle_frame.animate()
        graph_update_count += 1 

    # This statement is true when the graph_update_count counter variable is three.
    # If true then update the feedback graph and increase the counter by one.
    elif graph_update_count == 3:
        gui.app.feedback_frame.animate()
        graph_update_count += 1 

    # This statement is true when the graph_update_count counter variable is four.
    # If true then reset the graph_update_count counter to zero.
    if graph_update_count == 4:
        graph_update_count = 0

    # Update the GUI windows
    gui.app.update_idletasks()
    gui.app.update()    


    
