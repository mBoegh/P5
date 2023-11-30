"""
TO DO:
- Make UI with visualisation of detected mental command and its power (Eg. 'Lift' or 'Drop' and power 0-100)
"""

from EXONET.EXOLIB import JSON_Handler
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8, Int16, UInt16, Float32

from customtkinter import *
from customtkinter import StringVar, CTkSwitch 
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import math
import matplotlib.dates as mdates
from datetime import datetime, timedelta


class variables:
    """
    Class for initialising, setting and getting variables.
    When instanced in global space, then every other scope can access the variables.
    """
    
    def __init__(self):
        self.PWM_data = 75
        self.torque_data = 0
        self.RPM_data = 0
        self.eeg_data = 0
        self.current_angle = 90
        self.length = 4


class Gui(Node):
    """
    This is the GUI node of the EXONET ROS2 network.
    The purpose of the GUI is to make the prototype exoskeleton more usable, 
    and to aid in debugging and testing.
    """

    def __init__(self, timer_period, slider_zero, deadzone_low, deadzone_high, log_debug):

        # Initialising variables
        self.TIMER_PERIOD = timer_period
        self.SLIDER_ZERO = slider_zero
        self.DEADZONE_LOW = deadzone_low
        self.DEADZONE_HIGH = deadzone_high
        self.LOG_DEBUG = log_debug

        self.toggle_EEG_parameter = False

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('gui')

        self.get_logger().debug("Hello world!")
        self.get_logger().info("Hello world!")
        self.get_logger().warning("Hello world!")
        self.get_logger().error("Hello world!")
        self.get_logger().fatal("Hello world!")

        self.app = ParentWindow_MainMenu(None, self.get_logger())

        # Initialise variable msg as being of data type 'std_msgs.msg.Int8' imported as Int8
        self.msg = Int8()

        # Initialising a subscriber to the topic 'EEG_data'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.eeg_data_subscription = self.create_subscription(String, 'EEG_data', self.eeg_data_topic_callback, 10)
        self.eeg_data_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic 'Motor_signals'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
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

        # Initialising a publisher to the topic 'Manual_input_position_control_data'.
        # On this topic is published data of type std_msgs.msg.Int16 which is imported as Int16.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_input_velocity_control_data_publisher = self.create_publisher(Int16, 'Manual_input_velocity_control_data', 10)
        self.manual_input_velocity_control_data_publisher  # prevent unused variable warning

        # Initialising a publisher to the topic 'Manual_velocity_control_data'.
        # On this topic is published data of type std_msgs.msg.Int16 which is imported as Int16.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.manual_veloity_control_data_publisher = self.create_publisher(Int16, 'Manual_velocity_control_data', 10)
        self.manual_veloity_control_data_publisher  # prevent unused variable warning

        # Create a timer which periodically calls the specified callback function at a defined interval.
        # Initialise timer_counter as zero. This is iterated on each node spin
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)
        self.timer_counter = 0

        self.app.EEG_frame.animate()

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        self.app.update_idletasks()
        self.app.update()


    def eeg_data_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'eeg_data_subscription'
        """

        # Log info
        self.get_logger().debug(f"Received data: '{msg.data}'")

        self.app.exo_frame.PWMBar.set(msg[1]) # Set the progress bar to be filled a certain amount, needs to be between 0-1

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        self.app.update_idletasks()
        self.app.update()
        if self.app.position_control_window is not None:
            self.app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
        self.app.EEG_frame.animate()

    def motor_signals_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'motor_signals_subscription'
        """

        # Log info
        self.get_logger().debug(f"Recieved data: '{msg.data}'")

       # self.app.exo_frame.PWM_data = msg.data[0]
       # self.app.exo_frame.torque_data = msg.data[1]
       # self.app.exo_frame.RPM_data = msg.data[2]

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        self.app.update_idletasks()
        self.app.update()
        if self.app.position_control_window is not None:
            self.app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
        self.app.EEG_frame.animate()

    def feedback_joint_velocity_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'feedback_joint_velocity_subscription'
        """

        # Log info
        self.get_logger().debug(f"Recieved data '{msg.data}'")

        if self.app.position_control_window is not None:
            self.app.position_control_window.current_angle_label.configure(text=msg.data) # Update the content of the CurrentAngle Label

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        self.app.update_idletasks()
        self.app.update()
        if self.app.position_control_window is not None:
            self.app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
        self.app.EEG_frame.animate()


    def feedback_joint_angle_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription 'feedback_joint_angle_subscription'
        """
        
        # Log info
        self.get_logger().debug(f"Recieved data '{msg.data}'")

        if self.app.position_control_window is not None:
            self.app.position_control_window.current_angle_label.configure(text=msg.data) # Update the content of the CurrentAngle Label

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        self.app.update_idletasks()
        self.app.update()
        if self.app.position_control_window is not None:
            self.app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
        self.app.EEG_frame.animate()
        


    def timer_callback(self):
        """
        Function called at specific time interval, specified in 'settings.json'.
        """
        
        if self.toggle_EEG_parameter:
            # Load msg with current angle set in GUI 
            self.msg.data = data.current_angle

            # Publish msg using manual_control_data_publisher on topic 'Manual_control_data'
            self.manual_control_data_publisher.publish(self.msg)

            # Log info
            self.get_logger().debug(f"Published data: '{self.msg.data}'")

            # Iterate timer
            self.timer_counter += 1


class ParentWindow_MainMenu(CTk):
    """
    Main window of the GUI. Instances the Child windows.
    """

    def __init__(self, parent, logger):
        super().__init__(parent)

        self.logger = logger

        self.geometry("1200x800")
        self.parent = parent
        self.title("P5 GUI")
        self.mainWidgets()
        self.velocity_control_window = None
        self.position_control_window = None
        self.debug_menu_window = None

    def mainWidgets(self):
        """
        Calls and arranges all frames needed in the main window.
        Also makes and positiones the buttons which create child windows.
        """

        self.exo_frame = Frame_InfoExo(self, self.logger)
        self.manual_frame = Frame_MainMenu(self, self.logger)
        self.EEG_frame = Frame_VisualEegData(self, nb_points=100, logger=self.logger)

        self.exo_frame.grid(row= 0, column= 0, pady= 20, padx= 60)
        self.manual_frame.grid(row= 1, column= 0, pady= 20, padx= 60)
        self.EEG_frame.grid(row= 0, column= 1, pady=20, padx= 60)

        # The only way I could get the Debug window button to work
        # was by placing it here. Place it anywhere else,
        # and it will kick your brain by asking for more args than needed
        # for some reason. So leave it here


        self.position_control_button = CTkButton(master=self.manual_frame, text="Position Control", command=self.open_position_control_menu)
        self.position_control_button.grid(row= 1, column= 1, padx= 10, pady= 5)

        self.velocity_control_button = CTkButton(master=self.manual_frame, text="Velocity Control", command=self.open_velocity_control_menu)
        self.velocity_control_button.grid(row= 1, column= 2, padx= 10, pady= 5)


    def open_velocity_control_menu(self):
        """
        First chekcs if the menu exists (is open), 
        and if not then it creates the window. 
        Otherwise it lifts the window and sets the focus to it.
        """

        if gui.app.position_control_window:
            gui.app.position_control_window.destroy()

        if self.velocity_control_window is None or not self.velocity_control_window.winfo_exists():
            self.velocity_control_window = ChildWindow_VelocityControl(self.logger)
        else:
            self.velocity_control_window.focus()
            self.velocity_control_window.lift()


    def open_position_control_menu(self):
        """
        First chekcs if the menu exists (is open), 
        and if not then it creates the window. 
        Otherwise it lifts the window and sets the focus to it.
        """
        
        if gui.app.velocity_control_window: 
            gui.app.velocity_control_window.destroy()
    
        if self.position_control_window is None or not self.position_control_window.winfo_exists():
            self.position_control_window = ChildWindow_PositionControl(self.logger)
            self.visual_frame = Frame_VisualCurrentExoAngle(self.position_control_window, self.logger)
            self.visual_frame.grid(row= 4, column= 1, pady= 10, padx= 5)

        else:
            self.position_control_window.focus()
            self.position_control_window.lift()
        

class ChildWindow_VelocityControl(CTkToplevel):
    """
    This window contains everything related to manual velocity control.
    """

    def __init__(self, logger):

        CTkToplevel.__init__(self)
        self.geometry("400x300")  # Set the dimensions of the debug window

        self.logger = logger

        self.velocity_control_msg = Int16()
        self.input_velocity_control_msg = Int16()

        self.exit_button = CTkButton(self, text="Exit Button", command=self.exit_button_event)
        self.exit_button.grid(row=1, column=0, padx=10, pady=5)

        self.manual_stop_button = CTkButton(self, text="Stop", command=self.manual_stop_event)
        self.manual_stop_button.grid(row=2, column=0, padx=10, pady=5)

        self.slider = CTkSlider(self, from_=-20, to=20, command=self.slider_event, width=200, number_of_steps=40)
        self.slider.grid(row=2, column=1, padx=10, pady=5, columnspan=2)

        self.entry = CTkEntry(self,
            placeholder_text="Deg/sec",
            height=50,
            width=200,
            font=("Helvetica", 18),
            corner_radius=50,
            text_color="black",
            placeholder_text_color="grey",
            fg_color=("system", "white"),  # outer, inner
            state="normal",
        )
        self.entry.grid(row=4, column=1, padx=10, pady=5)

        submit_button = CTkButton(self, text="Submit", command= self.submit)
        submit_button.grid(row=5, column=1, padx=10, pady=5)

        # Bind the Enter key to the submit method
        self.entry.bind("<Return>", lambda event: self.submit())

    def exit_button_event(self):
        """
        Destroy the window, ie close the window.
        """
        self.destroy()


    def slider_event(self, value):
        """
        Function called whenever the slider is manipulated.
        Handles deadzone, and publishes slider value to topic
        'Manual_velocity_control_data'.
        """

        # Slider deadzone
        #if value > gui.DEADZONE_LOW and value < gui.DEADZONE_HIGH:
        #    value = gui.SLIDER_ZERO

        self.velocity_control_msg.data = int(value)

        gui.manual_veloity_control_data_publisher.publish(self.velocity_control_msg)

        self.logger.debug(f"Target velocity: {value}")


    def manual_stop_event(self):
        """
        Function called whenever the Stop button is pressed.
        Sets the slider to 0, thereby stopping the motor.
        """

        self.slider.set(gui.SLIDER_ZERO)

        self.slider_event(gui.SLIDER_ZERO)
        self.logger.info("Stopped")


    def clear(self):
        """
        Clears the entry field of any characters.
        """

        self.entry.delete(0, END)

    def submit(self):
        """
        Publishes integer values in the entry field.
        Can be triggered with 'RETURN' button.
        """

        value = int(self.entry.get())

        self.input_velocity_control_msg.data = value
        
        try:
            gui.manual_input_velocity_control_data_publisher.publish(self.input_velocity_control_msg)
            self.logger.debug(f"Published data: '{self.input_velocity_control_msg.data}'")

        except Exception as e:
            self.logger.warning(f"Failed to publish data: '{self.input_velocity_control_msg.data}' With error: {e}")

        self.clear()


class ChildWindow_PositionControl(CTkToplevel):
    """
    This window contains everything related to manual position control.
    """

    def __init__(self, logger):

        CTkToplevel.__init__(self)
        self.geometry("400x300") # Set the dimensions of the debug window

        self.logger = logger

        self.position_control_msg = UInt16()

        # Initializes the label which shows the current angle of the exo skeleton
        self.current_angle_label = CTkLabel(self, text= str(data.current_angle))
        self.manual_down_button = CTkButton(self, text="v", command= self.manual_down_event)
        self.manual_up_button = CTkButton(self, text="^", command= self.manual_up_event)
        
        self.manual_up_button.grid(row= 2, column= 0, padx= 10, pady= 5)
        self.manual_down_button.grid(row= 2, column= 2, padx= 10, pady= 5)

        # Places the label which shows the current angle, and makes it the width of the above 2 buttons
        self.current_angle_label.grid(row= 3, column= 1, padx= 10, pady= 5)

        self.exit_button = CTkButton(self, text="Exit Button", command= self.exit_button_event)
        self.exit_button.grid(row= 0, column= 0, padx= 10, pady= 5)

    ##  label = CTkLabel(self, text="", font=("Helvetica", 24))
    ##  label.grid(row=4, column=0, padx=10, pady=5)

        self.entry = CTkEntry(self,
            placeholder_text="Angle (degrees)",
            height=50,
            width=200,
            font=("Helvetica", 18),
            corner_radius=50,
            text_color="black",
            placeholder_text_color="grey",
            fg_color=("system", "white"),  # outer, inner
            state="normal",
        )
        self.entry.grid(row=2, column=1, padx=10, pady=5)

        submit_button = CTkButton(self, text="Submit", command= self.submit)
        submit_button.grid(row=1, column=1, padx=10, pady=5)

        # Bind the Enter key to the submit method
        self.entry.bind("<Return>", lambda event: self.submit())


    # Define Functions used in the Manual Control frame
    def manual_up_event(self):
        """
        Function called when the '^' button is pressed.
        Increases the target joint angle with one degree.
        This is where the upper joint angle limit is set.
        """
        
        # If the upper limit is reached, exit function
        if (data.current_angle == 170): return
        data.current_angle += 1
        gui.app.position_control_window.current_angle_label.configure(text= data.current_angle) # Update the content of the CurrentAngle Label

        self.position_control_msg.data = data.current_angle

        gui.manual_position_control_data_publisher.publish(self.position_control_msg)
        
        self.logger.debug(f"Published data: '{self.position_control_msg.data}'")


    def manual_down_event(self):
        """
        Function called when the 'v' button is pressed.
        Decreases the target joint angle with one degree.
        This is where the lower joint angle limit is set.
        """
        
        # If the lower limit is reached, exit function
        if (data.current_angle == 40): return 
        data.current_angle -= 1
        gui.app.position_control_window.current_angle_label.configure(text= data.current_angle) # Update the content of the CurrentAngle Label
        
        self.position_control_msg.data = data.current_angle

        gui.manual_position_control_data_publisher.publish(self.position_control_msg)

        self.logger.debug(f"Published data: '{self.position_control_msg.data}'")


    # Destroy the pop up menu window, ie close the window
    def exit_button_event(self): 
        """
        Destroy the window, ie close the window.
        """

        self.destroy()


    def clear(self):
        """
        Clears the entry field of any characters.
        """

        self.entry.delete(0, END)

    def submit(self):
        """
        Publishes integer values in the entry field.
        Can be triggered with 'RETURN' button.
        """

        value = int(self.entry.get())

        if (value <= 40): 
            data.current_angle = 40 

        elif (value >= 170):
            data.current_angle = 170
        
        else:
            data.current_angle = value

        gui.app.position_control_window.current_angle_label.configure(text= data.current_angle) # Update the content of the CurrentAngle Label
        

        self.position_control_msg.data = data.current_angle
        
        try:
            gui.manual_position_control_data_publisher.publish(self.position_control_msg)
            self.logger.debug(f"Published data: '{self.position_control_msg.data}'")

        except Exception as e:
            self.logger.warning(f"Failed to publish data: '{self.position_control_msg.data}' With error: {e}")

        self.clear()


class Frame_MainMenu(CTkFrame):
    """
    Frame wherin the main menu controls are placed.
    """

    def __init__(self, parent, logger):
        super().__init__(parent)

        self.logger = logger

        self.parent = parent
        self.widgets()


    def widgets(self):
        """
        Function which initializes and places all the used widgets in the frame.
        """
        
        self.switch_var = StringVar(value="False")
        self.switch = CTkSwitch(self, text="EEG", command= self.eeg_toggle,
                                variable=self.switch_var, onvalue="True", offvalue="False")
        self.switch.grid(row=1, column= 0, padx= 10, pady= 5)

    def eeg_toggle(self):
        """
        Function called when the EEG switch is toggled.
        The switch state determines wether or not the system shall attempt
        to connect with the EEG Node-Red system on TCP.
        When a connection is established, then this determines wether or not
        the system reacts to the EEG data stream.
        """

        msg = Bool()

        value = gui.app.manual_frame.switch_var.get()

        if value == "True":
            gui.app.position_control_button.configure(state= DISABLED)
            gui.app.velocity_control_button.configure(state= DISABLED)
            
            if gui.app.position_control_window:
                gui.app.position_control_window.destroy()


            if gui.app.velocity_control_window: 
                gui.app.velocity_control_window.destroy()
   

            msg.data = True
            
            gui.toggle_EEG_parameter == True

        else:
            gui.app.position_control_button.configure(state= NORMAL)
            gui.app.velocity_control_button.configure(state= NORMAL)
            
            msg.data = False
            
            gui.toggle_EEG_parameter == False

        gui.eeg_toggle_publisher.publish(msg)


class Frame_InfoExo(CTkFrame):
    """
    Frame wherein information about the current state of the exo skeleton is shown.
    """

    def __init__(self, parent, logger):
        CTkFrame.__init__(self, parent)

        self.logger = logger

        self.parent = parent
        self.widgets()

    def widgets(self):
        """
        All the used widgets are initialized and placed in the frame here.
        """
        
        # Text Labels
        self.PWMLabel = CTkLabel(self, text="PWM: ")
        self.TorqueLabel = CTkLabel(self, text="Torque: ")
        self.RPMLabel = CTkLabel(self, text="Motor RPM: ")
        self.PWMBar = CTkProgressBar(self, orientation="horizontal")

        # Data Labels
        self.PWMDataLabel = CTkLabel(self, text= str(data.PWM_data))
        self.TorqueDataLabel = CTkLabel(self, text= str(data.torque_data))
        self.RPMDataLabel = CTkLabel(self, text= str(data.RPM_data))

        # Placing the widgets on the grid in the ExoFrame frame
        self.TorqueDataLabel.grid(row= 1, column=1, padx=10, pady=5)
        self.TorqueLabel.grid(row= 1, column= 0, padx= 10, pady= 5)
        self.RPMLabel.grid(row=2, column= 0, padx=10, pady=5)
        self.RPMDataLabel.grid(row= 2, column= 1, padx= 10, pady= 5)
        self.PWMBar.grid(row= 0, column= 1, padx= 10, pady= 5)
        self.PWMDataLabel.grid(row= 0, column=2, padx= 10, pady= 5)
        self.PWMLabel.grid(row= 0, column= 0, padx= 10, pady= 5)


class Frame_VisualEegData(CTkFrame):
    """
    Makes and displays the graph for the EEG data, the class will need to be given how many data mounts
    which should be displayed on the graph.
    """

    def __init__(self, parent, nb_points, logger):
        super().__init__(parent)

        self.logger = logger

        self.parent = parent
        self.widgets(nb_points)

    def widgets(self, nb_points):
        # Define the graph, and configure the axes
        self.figure, self.ax = plt.subplots(figsize=(5,3), dpi=50)
        # format the x-axis to show the time
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

        # initial x and y data
        date_time_obj = datetime.now() + timedelta(seconds=-nb_points)
        self.x_data = [date_time_obj + timedelta(seconds=i) for i in range(nb_points)]
        self.y_data = [0 for i in range(nb_points)]
        #create the first plot
        self.plot = self.ax.plot(self.x_data, self.y_data, label='EEG data')[0]
        self.ax.set_ylim(0,100)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])

        FrameTopLabel = CTkLabel(self, text="EEG Data")
        FrameTopLabel.pack(pady=10, padx=10, side='top')
        self.canvas = FigureCanvasTkAgg(self.figure, self)
        self.canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=True)

    def animate(self):
        #append new data point to x and y data
        self.x_data.append(datetime.now())
        self.y_data.append(int(data.eeg_data))
        #remove oldest datapoint
        self.x_data = self.x_data[1:]
        self.y_data = self.y_data[1:]
        #update plot data
        self.plot.set_xdata(self.x_data)
        self.plot.set_ydata(self.y_data)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])
        self.canvas.draw_idle() #redraw plot


class Frame_VisualCurrentExoAngle(CTkFrame):
    """
    Frame wherein the current angle of the exoskeleton is illustrated.
    """

    def __init__(self, parent, logger):
        super().__init__(parent)

        self.logger = logger

        self.parent = parent
        # Call the draw function
        self.draw()

    def draw(self):
        """
        Handles the initial drawing of the visualization of the current configuration of the exoskeleton,
        and ends by redrawing the canvas(figure).
        """
        
        endx = 2 + data.length * math.cos(math.radians((data.current_angle-90))) # Calculate the end point for the movable arm
        endy = 5 + data.length * -math.sin(math.radians(data.current_angle-90))

        self.figure, self.ax = plt.subplots(figsize=(3,3), dpi=50) # Create the figure without content
        self.ax.set_ylim(0,10) # Set the limits of the axes in the plot
        self.ax.set_xlim(0,10)
        self.grap = self.ax.plot([2,2,endx], [9,5,endy], 'bo-') # Draw the plot in the figure

        self.canvas = FigureCanvasTkAgg(self.figure, self) # Sets the figure to be a canvas, such it can be drawn by tkinter
        self.canvas.get_tk_widget().pack(side='top', fill=BOTH, expand=True) # Place the canvas in the frame

    
    def animate(self):
        """
        Used to redraw the plot, needs to recalculate the end points for the movable arm.
        """
        
        endx = 2 + data.length * math.cos(math.radians((data.current_angle-90)))
        endy = 5 + data.length * -math.sin(math.radians(data.current_angle-90))

        plt.cla() # Clears all content on the plot, without removing the axes
        self.ax.set_ylim(0,10) # Redefine the limits of the plot
        self.ax.set_xlim(0,10)
        #self.grap.remove()
        self.grap = self.ax.plot([2,2,endx], [9,5,endy], 'bo-') # Redraw the exoskeleton visualization
        
        self.canvas.draw_idle() # And redraw the canvas


####################
####    MAIN    ####
####################

# Path for 'settings.json' file
json_file_path = ".//src//EXONET//EXONET//settings.json"

# Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
handler = JSON_Handler(json_file_path)

# Get settings from 'settings.json' file
TIMER_PERIOD = handler.get_subkey_value("gui", "TIMER_PERIOD")
SLIDER_ZERO = handler.get_subkey_value("gui", "SLIDER_ZERO")
DEADZONE_LOW = handler.get_subkey_value("gui", "DEADZONE_LOW")
DEADZONE_HIGH = handler.get_subkey_value("gui", "DEADZONE_HIGH")
LOG_DEBUG = handler.get_subkey_value("gui", "LOG_DEBUG")
LOG_LEVEL = handler.get_subkey_value("gui", "LOG_LEVEL")

# Change appearance of the GUI
set_appearance_mode('system')
set_default_color_theme("blue")

data = variables()

# Initialize the rclpy library
rclpy.init()

rclpy.logging.set_logger_level("gui", eval(LOG_LEVEL))

# Instance the node class
gui = Gui(TIMER_PERIOD, SLIDER_ZERO, DEADZONE_LOW, DEADZONE_HIGH, LOG_DEBUG)


while True:
    # Begin looping the node
    rclpy.spin_once(gui, timeout_sec=0.01)

    gui.app.exo_frame.PWMBar.set(data.PWM_data) # Set the progress bar to be filled a certain amount, needs to be between 0-1

    if gui.app.position_control_window is not None:
        gui.app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
    gui.app.EEG_frame.animate()

    gui.app.update_idletasks()
    gui.app.update()     

    
