import customtkinter as ctk
import time

# Creates our root window, from here all content will be set into
ctk.set_appearance_mode('dark')
ctk.set_default_color_theme("green")
root = ctk.CTk()
root.geometry("1000x500")

# Sets the window name
root.title("Interface for Exoskeleton system")


#### All of our data from the currently running system
#### Has right now been hardcoded just for checking it works
TorqueData = 500
dataPWM = 0.75
RPMData = 9001

#### Initializing all the frames in the window "root"
frameExo = ctk.CTkFrame(master=root)
frameEEG = ctk.CTkFrame(master=root)
framePosition = ctk.CTkFrame(master=root)
frameManualControl = ctk.CTkFrame(master=root)

#### Placing all the frames in the window "root"
frameExo.grid(row=0, column= 0, pady= 20,padx= 60)
frameManualControl.grid(row=1, column= 0, pady= 20,padx= 60)
frameEEG.grid(row=0, column= 1,pady= 20,padx= 60)
framePosition.grid(row=1, column= 1, pady= 20,padx= 60)

# Exo Labels
if True:
    ###### All info related to the exoskeleton, 
    ###### such as PWM, torque and so on
    # Initialize the labels, formatting happens afterwards
    PWMLabel = ctk.CTkLabel(master=frameExo, text= "PWM: ")
    TorqueLabel = ctk.CTkLabel(master=frameExo, text= "Torque: ")
    RPMLabel = ctk.CTkLabel(master=frameExo, text= "RPM: ")

    # Formating for the labels, ie. where they are located on the page
    PWMLabel.grid(row= 0, column=0, padx= 10, pady= 5)
    TorqueLabel.grid(row= 1, column=0,padx= 10, pady= 5)
    RPMLabel.grid(row= 2, column=0, padx= 10, pady= 5)
    PWMBar = ctk.CTkProgressBar(master=frameExo, orientation="horizontal")

    # Initilizing the Data sections
    PWMDataLabel = ctk.CTkLabel(master=frameExo, text= dataPWM)
    TorqueDataLabel = ctk.CTkLabel(master=frameExo, text= TorqueData)
    RPMDataLabel = ctk.CTkLabel(master=frameExo, text= RPMData)

    # Placing data labels in the frame
    PWMBar.grid(row= 0, column= 1, padx= 10, pady= 5)
    PWMDataLabel.grid(row= 0, column= 2, padx= 10, pady= 5)
    TorqueDataLabel.grid(row= 1, column= 1, padx= 10, pady= 5)
    RPMDataLabel.grid(row= 2, column= 1, padx= 10, pady= 5)

# Manual Control labels and variables
CurrentAngleManualControl = 41
IsDebugOpen = False


############ TO-DO ###############
# Raise the flag for destroying the debug menu inside the ExitButton() function
# and check outside of the OpenTopLevel() function whether the debug should be
# destroyed or not

if True:
 
    # Define Functions used in the Manual Control frame
    def ManualUpEvent():
        global CurrentAngleManualControl
        # If the upper limit is reached, exit function
        if (CurrentAngleManualControl == 170): return
        CurrentAngleManualControl += 1

    def ManualDownEvent():
        global CurrentAngleManualControl
        # If the lower limit is reached, exit function
        if (CurrentAngleManualControl == 40): return 
        CurrentAngleManualControl -= 1

    def OpenTopLevel():
        global IsDebugOpen
        
        if (IsDebugOpen == True): return # Do nothing is debug menu is open

        # Funtion to close the debug menu
        def ExitButton():
            global IsDebugOpen
            
            debug.destroy()
            time.sleep(1)
            IsDebugOpen = False

        # Open Debug menu
        debug = ctk.CTk()
        debug.geometry("200x200")
        IsDebugOpen = True

        # Initialize the exit button and place it in debug window
        ExitbuttonButton = ctk.CTkButton(master=debug, text= "Exit Button", command=ExitButton)
        ExitbuttonButton.grid(row= 0, column= 0, padx= 10, pady= 10)
        

        

    # Initialize all labels and buttons
    CurrentAngleManualLabel = ctk.CTkLabel(master=frameManualControl, text= CurrentAngleManualControl)

    ManualUpButton = ctk.CTkButton(master=frameManualControl, text= "^", command=ManualUpEvent)
    ManualDownButton = ctk.CTkButton(master=frameManualControl, text= "v", command=ManualDownEvent)
    DebugButton = ctk.CTkButton(master=frameManualControl, text= "Debug Menu", command=OpenTopLevel)

    # Format the buttons in the frame "Manual Control"
    ManualUpButton.grid(row= 0, column= 0, padx= 10, pady=5)
    ManualDownButton.grid(row= 0, column= 1, padx= 10, pady=5)
    DebugButton.grid(row= 0, column= 2, padx= 20, pady=5)
    
    CurrentAngleManualLabel.grid(row=1, column= 0, padx= 10, pady=5, columnspan=2)




while(True):
    # Updating the data in the labels
    PWMDataLabel.configure(text= dataPWM)
    TorqueDataLabel.configure(text= TorqueData)
    RPMDataLabel.configure(text= RPMData)

    # Inputting the actual data into the progessbar
    PWMBar.set(dataPWM)

    CurrentAngleManualLabel.configure(text= CurrentAngleManualControl)
 
    # Update the window
    root.update_idletasks()
    root.update()
    time.sleep(0.25)