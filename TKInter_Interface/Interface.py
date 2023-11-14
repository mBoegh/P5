from customtkinter import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import math
import matplotlib.dates as mdates
from datetime import datetime, timedelta
from psutil import cpu_percent

# Change appearance of the GUI
set_appearance_mode('system')
set_default_color_theme("blue")


class Variables():
    """One centralized place for data to be stored"""
    RPM_data = 20
    torque_data = 100
    length = 4
    current_angle = 170
data = Variables()

class ManualControl(CTkFrame):
    """Class for the manual control frame in the main window"""
    data = Variables() # Calling variables to make them accessible in the manual frame

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.widgets()

    def widgets(self):
        """Function which initializes and places all the used widgets in the manual control frame"""
        # Initializes the label which shows the current angle of the exo skeleton
        self.current_angle_label = CTkLabel(self, text= str(self.data.current_angle))

        # Initializes the buttons for manually controlling the exo angle
        self.manual_down_button = CTkButton(self, text="v", command= self.manual_down_event)
        self.manual_up_button = CTkButton(self, text="^", command= self.manual_up_event)

        # Places the above buttons in the manual control frame
        self.manual_up_button.grid(row= 0, column= 0, padx= 10, pady= 5)
        self.manual_down_button.grid(row= 0, column= 1, padx= 10, pady= 5)
        
        # Places the label which shows the current angle, and makes it the width of the above 2 buttons
        self.current_angle_label.grid(row= 1, column= 0, padx= 10, pady= 5, columnspan=2)

    # Define Functions used in the Manual Control frame
    def manual_up_event(self):
        
        # If the upper limit is reached, exit function
        if (data.current_angle == 170): return
        data.current_angle += 1

    def manual_down_event(self):
        
        # If the lower limit is reached, exit function
        if (data.current_angle == 40): return 
        data.current_angle -= 1


class EEG(CTkFrame):
    """Makes and displays the graph for the EEG data, the class will need to be given how many data mounts
    which should be displayed on the graph"""

    def __init__(self, parent, nb_points):
        super().__init__(parent)
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
        self.y_data.append(int(cpu_percent()))
        #remove oldest datapoint
        self.x_data = self.x_data[1:]
        self.y_data = self.y_data[1:]
        #update plot data
        self.plot.set_xdata(self.x_data)
        self.plot.set_ydata(self.y_data)
        self.ax.set_xlim(self.x_data[0], self.x_data[-1])
        self.canvas.draw_idle() #redraw plot


class Exo(CTkFrame):
    data = Variables() # Making the live data accesible in the Exoskeleton frame

    def __init__(self, parent):
        CTkFrame.__init__(self, parent)
        self.parent = parent
        self.widgets()

    def widgets(self):
        """All the used widgets are initialized and placed in the frame here"""
        # Text Labels
        self.PWMLabel = CTkLabel(self, text="PWM: ")
        self.TorqueLabel = CTkLabel(self, text="Torque: ")
        self.RPMLabel = CTkLabel(self, text="Motor RPM: ")
        self.PWMBar = CTkProgressBar(self, orientation="horizontal")

        # Data Labels
        self.PWMDataLabel = CTkLabel(self, text= "0.75")
        self.TorqueDataLabel = CTkLabel(self, text= str(self.data.torque_data))
        self.RPMDataLabel = CTkLabel(self, text= str(self.data.RPM_data))

        # Placing the widgets on the grid in the Exo frame
        self.TorqueDataLabel.grid(row= 1, column=1, padx=10, pady=5)
        self.TorqueLabel.grid(row= 1, column= 0, padx= 10, pady= 5)
        self.RPMLabel.grid(row=2, column= 0, padx=10, pady=5)
        self.RPMDataLabel.grid(row= 2, column= 1, padx= 10, pady= 5)
        self.PWMBar.grid(row= 0, column= 1, padx= 10, pady= 5)
        self.PWMDataLabel.grid(row= 0, column=2, padx= 10, pady= 5)
        self.PWMLabel.grid(row= 0, column= 0, padx= 10, pady= 5)


class DebugMenu(CTkToplevel):
    """Content for the debug menu, aswell as generating the window 
    where the content is contained"""

    def __init__(self):
        CTkToplevel.__init__(self)
        self.geometry("400x300") # Set the dimensions of the debug window

        # Destroy the Debug menu window, ie close the window
        def exit_button_event(): self.destroy()

        self.debug_menu_label = CTkLabel(self, text="Debug Menu")
        self.debug_menu_label.grid(row=0, column= 0, padx= 10, pady= 5)

        self.exit_button = CTkButton(self, text="Exit Button", command= exit_button_event)
        self.exit_button.grid(row= 1, column= 0, padx= 10, pady= 5)


class Visual(CTkFrame):
    data = Variables()

    # Initialize the frame
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        # Call the draw function
        self.draw()

    def draw(self):
        """Handles the initial drawing of the visualization of the current configuration of the exoskeleton,
        and ends by redrawing the canvas(figure)"""
        endx = 2 + self.data.length * math.cos(math.radians((data.current_angle-90))) # Calculate the end point for the movable arm
        endy = 5 + self.data.length * -math.sin(math.radians(data.current_angle-90))

        self.figure, self.ax = plt.subplots(figsize=(3,3), dpi=50) # Create the figure without content
        self.ax.set_ylim(0,10) # Set the limits of the axes in the plot
        self.ax.set_xlim(0,10)
        self.grap = self.ax.plot([2,2,endx], [9,5,endy], 'bo-') # Draw the plot in the figure

        self.canvas = FigureCanvasTkAgg(self.figure, self) # Sets the figure to be a canvas, such it can be drawn by tkinter
        self.canvas.get_tk_widget().pack(side='top', fill=BOTH, expand=True) # Place the canvas in the frame

    
    def animate(self):
        """Used to redraw the plot, needs to recalculate the end points for the movable arm"""
        endx = 2 + self.data.length * math.cos(math.radians((data.current_angle-90)))
        endy = 5 + self.data.length * -math.sin(math.radians(data.current_angle-90))

        plt.cla() # Clears all content on the plot, without removing the axes
        self.ax.set_ylim(0,10) # Redefine the limits of the plot
        self.ax.set_xlim(0,10)
        #self.grap.remove()
        self.grap = self.ax.plot([2,2,endx], [9,5,endy], 'bo-') # Redraw the exoskeleton visualization
        
        self.canvas.draw_idle() # And redraw the canvas


class MainW(CTk):
    def __init__(self, parent):
        super().__init__(parent)
        self.geometry("1200x800")
        self.parent = parent
        self.title("P5 GUI")
        self.mainWidgets()
        self.toplevel_window = None

    def mainWidgets(self):
        """Calls and arranges all frames needed in the main window"""
        self.exo_frame = Exo(self)
        self.manual_frame = ManualControl(self)
        self.EEG_frame = EEG(self, nb_points=100)
        self.visual_frame = Visual(self)

        self.exo_frame.grid(row= 0, column= 0, pady= 20, padx= 60)
        self.manual_frame.grid(row= 1, column= 0, pady= 20, padx= 60)
        self.EEG_frame.grid(row= 0, column= 1, pady=20, padx= 60)
        self.visual_frame.grid(row= 1, column= 1, pady= 0, padx= 0)

        # The only way I could get the Debug window button to work
        # was by placing it here. Place it anywhere else,
        # and it will kick your brain by asking for more args than needed
        # for some reason. So leave it here
        self.debug_button = CTkButton(master=self.manual_frame, text="Debug Menu", command=self.open_top_level)
        self.debug_button.grid(row= 0, column= 2, padx= 10, pady= 5)

    # 
    def open_top_level(self):
        """First chekcs if the debug menu exists (is open), and if it isnt
        Then it creates the window. Or if it does exist, 
        then it lifts the window and sets the focus to it"""
        if self.toplevel_window is None or not self.toplevel_window.winfo_exists():
            self.toplevel_window = DebugMenu()
        else:
            self.toplevel_window.focus()
            self.toplevel_window.lift()


if __name__=="__main__":
    # Calling our main class, since class is for the main window
    # The argument is None, since it does not have any parents
    app = MainW(None)
    data = Variables()

    # We need this while(True) such that we can update the values on screen
    # Thus all widgets(Values, Progressbar, EEG Graph) needs to be updated in this loop
    while(True):
        app.exo_frame.PWMBar.set(0.75) # Set the progress bar to be filled a certain amount, needs to be between 0-1
        app.manual_frame.current_angle_label.configure(text=data.current_angle) # Update the content of the CurrentAngle Label
        app.visual_frame.animate() # Redraws the frame which contains the Exoskeleton visualization
        app.EEG_frame.animate()

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        app.update_idletasks()
        app.update()
        

