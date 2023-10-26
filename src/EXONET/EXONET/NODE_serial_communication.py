"""
TO DO:
 - Get settings from 'NODE_settings.py' on topic 'settings/NODE_serial_communication'
 - Recieve motor signals data from topic 'Motor_signals'
 - Send recieved motor signals data as is to Arduino across serial connection using class 'serial2arduino'
"""

import serial
import time

class serial2arduino:
    """
    Class for establishing serial communication between Python script and an Arduino. 
    Takes parameters:
       serial_port - the COM port connection with USB)
       baud_rate - defaults 9600
       timeout - time before connection attempt is abandonded in seconds)
       Debug - bool print statements (defaults False)
    """

    def __init__(self, serial_port, baud_rate=9600, timeout=1, Debug=False):

         # init variables
        self.SERIAL_PORT = serial_port
        self.BAUD_RATE = baud_rate
        self.TIMEOUT = timeout
        self.DEBUG = Debug


    def establish_connection(self):
        """
        Function for handling establishing connection between Python pyserial and Arduino.
        """

        # Open a serial connection
        self.arduino = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, self.TIMEOUT)

        # Wait for the Arduino to initialize
        time.sleep(2)

    def send_data(self, data):
        """
        Function for handling sending of data across serial connection established in function 'establish_connection'.
        """

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ class 'serial2arduino' function 'Send_data'; VARIABLE 'data': {data}")
    
        # Encode data as encoded_data
        encoded_data = data.encode()

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ class 'serial2arduino' function 'Send_data'; VARIABLE 'encoded_data': {encoded_data}")

        # Send encoded_data Arduino
        self.arduino.write(encoded_data)


####################
######  MAIN  ######
####################


def main():
    print("Hello world!")
    
    SERIAL_PORT = None
    BAUD_RATE = None
    TIMEOUT = None
    DEBUG = None

    # Instance the serial2arduino class
    arduino = serial2arduino(SERIAL_PORT, BAUD_RATE, TIMEOUT, DEBUG)

if __name__ == "__main__":
    main()
