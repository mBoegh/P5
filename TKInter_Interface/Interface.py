from customtkinter import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import math

set_appearance_mode('dark')
set_default_color_theme("green")

class variables():
    """One centralized place for data to be stored"""
    RPMData = 20
    TorqueData = 100

    length = 4
    CurrentAngleManualControl = 170
data = variables()

class manualControl(CTkFrame):
    """Class for the manual control frame in the main window"""
    data = variables() # Calling variables to make them accessible in the manual frame
    def __init__(self, parent):
        CTkFrame.__init__(self, parent)
        self.parent = parent
        self.widgets()

    def widgets(self):
        """Function which initializes and places all the used widgets in the manual control frame"""
        # Initializes the label which shows the current angle of the exo skeleton
        self.CurrentAngleManualLabel = CTkLabel(self, text= data.CurrentAngleManualControl)

        # Initializes the buttons for manually controlling the exo angle
        self.ManualDownButton = CTkButton(self, text="v", command=self.ManualDownEvent)
        self.ManualUpButton = CTkButton(self, text="^", command=self.ManualUpEvent)

        # Places the above buttons in the manual control frame
        self.ManualUpButton.grid(row= 0, column= 0, padx= 10, pady= 5)
        self.ManualDownButton.grid(row= 0, column= 1, padx= 10, pady= 5)
        
        # Places the label which shows the current angle, and makes it the width of the above 2 buttons
        self.CurrentAngleManualLabel.grid(row= 1, column= 0, padx= 10, pady= 5, columnspan=2)

    # Define Functions used in the Manual Control frame
    def ManualUpEvent(self):
        
        # If the upper limit is reached, exit function
        if (data.CurrentAngleManualControl == 170): return
        data.CurrentAngleManualControl += 1

    def ManualDownEvent(self):
        
        # If the lower limit is reached, exit function
        if (data.CurrentAngleManualControl == 40): return 
        data.CurrentAngleManualControl -= 1

class EEG(CTkFrame):
    data = variables() # Making the live data accesible in the EEG frame
    def __init__(self, parent):
        CTkFrame.__init__(self, parent)
        self.parent = parent
        #self.widgets()

class exo(CTkFrame):
    data = variables() # Making the live data accesible in the Exoskeleton frame
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
        self.TorqueDataLabel = CTkLabel(self, text= self.data.TorqueData)
        self.RPMDataLabel = CTkLabel(self, text= self.data.RPMData)

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
        self.geometry("400x300")

        def ExitButtonEvent():
            global IsDebugOpen
            
            self.destroy()
            IsDebugOpen = False

        self.Label1 = CTkLabel(self, text="Debug Menu")
        self.Label1.grid(row=0, column= 0, padx= 10, pady= 5)

        self.ExitButton = CTkButton(self, text="Exit Button", command= ExitButtonEvent)
        self.ExitButton.grid(row= 1, column= 0, padx= 10, pady= 5)


class Visual(CTkFrame):

    # Initialize the frame
    def __init__(self, parent):
        CTkFrame.__init__(self, parent)
        self.parent = parent
        # Call the draw function
        self.draw()

    def draw(self):
        """Handles the initial drawing of the visualization of the current configuration of the exoskeleton,
        and ends by redrawing the canvas(figure)"""
        endx = 2 + data.length * math.cos(math.radians((data.CurrentAngleManualControl-90))) # Calculate the end point for the movable arm
        endy = 2 + data.length * -math.sin(math.radians(data.CurrentAngleManualControl-90))

        self.figure = plt.figure(figsize=(5,5), dpi=100) # Create the figure without content
        self.ax = self.figure.add_subplot(111) # Add a plot to the above figure
        self.ax.set_ylim(-3,10) # Set the limits of the axes in the plot
        self.ax.set_xlim(-3,10)
        self.grap = self.ax.plot([2,2,endx], [6,2,endy], 'ro-') # Draw the plot in the figure

        self.canvas = FigureCanvasTkAgg(self.figure, self) # Sets the figure to be a canvas, such it can be drawn by tkinter
        self.canvas.get_tk_widget().pack(side='top', fill=BOTH, expand=True) # Place the canvas in the frame
        self.canvas.draw_idle() # redraw the canvas
    
    def animate(self):
        """Used to redraw the plot, needs to recalculate the end points for the movable arm"""
        endx = 2 + data.length * math.cos(math.radians((data.CurrentAngleManualControl-90)))
        endy = 2 + data.length * -math.sin(math.radians(data.CurrentAngleManualControl-90))

        plt.cla() # Clears all content on the plot, without removing the axes
        self.ax.set_ylim(-3,10) # Redefine the limits of the plot
        self.ax.set_xlim(-3,10)
        #self.grap.remove()
        self.grap = self.ax.plot([2,2,endx], [6,2,endy], 'ro-') # Redraw the exoskeleton visualization
        
        self.canvas.draw_idle() # And redraw the canvas








class MainW(CTk):
    def __init__(self, parent):
        CTk.__init__(self, parent)
        self.geometry("1000x500")
        self.parent = parent
        self.title("P5 GUI")
        self.mainWidgets()
        self.toplevel_window = None

    def mainWidgets(self):
        """Calls and arranges all frames needed in the main window"""
        self.ExoFrame = exo(self)
        self.ManualFrame = manualControl(self)
        self.EEGFrame = EEG(self)
        self.VisualFrame = Visual(self)

        self.ExoFrame.grid(row= 0, column= 0, pady= 20, padx= 60)
        self.ManualFrame.grid(row= 1, column= 0, pady= 20, padx= 60)
        self.EEGFrame.grid(row= 0, column= 1, pady=20, padx= 60)
        self.VisualFrame.grid(row= 1, column= 1, pady= 0, padx= 0)

        # The only way I could get the Debug window button to work
        # was by placing it here. Place it anywhere else,
        # and it will kick your brain by asking for more args than needed
        # for some reason. So leave it here
        self.DebugButton = CTkButton(master=self.ManualFrame, text="Debug Menu", command=self.OpenTopLevel)
        self.DebugButton.grid(row= 0, column= 2, padx= 10, pady= 5)

    # 
    def OpenTopLevel(self):
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

    # We need this while(True) such that we can update the values on screen
    # Thus all widgets(Values, Progressbar, EEG Graph) needs to be updated in this loop
    while(True):
        app.ExoFrame.PWMBar.set(0.75) # Set the progress bar to be filled a certain amount, needs to be between 0-1
        app.ManualFrame.CurrentAngleManualLabel.configure(text= data.CurrentAngleManualControl) # Update the content of the CurrentAngle Label
        app.VisualFrame.animate() # Redraws the frame which contains the Exoskeleton visualization

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        app.update_idletasks()
        app.update()
        

