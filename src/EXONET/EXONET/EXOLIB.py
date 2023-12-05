import json
import socket
import serial
import serial.tools.list_ports
import time
from collections import deque
import numpy as np



class JSON_Handler:
    """
    Class for handling loading keys for each respective ROS2 node, subkeys for each setting and the value of each setting. 
    """
    
    def __init__(self, json_file_path):
        
        self.json_file_path = json_file_path
        self.json_obj = None

        try:
            self.json_obj = json.load(open(self.json_file_path, 'r'))
        except json.JSONDecodeError as error:
            print(f"DEBUG @ script 'EXOLIB.py' class 'JSON_Handler' function '__init__'; SYSTEM MESSAGE: Failed loading .json file with error: {error}")
            return None  # Return None if the input is not valid JSON


    def get_keys(self):
        """
        Function for getting all top level keys in a json file.
        """

        if isinstance(self.json_obj, dict):
            return list(self.json_obj.keys())
        else:                
            return None  # Return None if the input is not a JSON object
    

    def get_sublevel_keys(self, key):
        """
        Function for getting all sublevel keys of a specified key in a json file.
        """
            
        if isinstance(self.json_obj, dict):
            if key in self.json_obj and isinstance(self.json_obj[key], dict):
                return list(self.json_obj[key].keys())
            else:
                return None  # Sublevel key not found or is not a dictionary
        else:
            return None  # Input is not a JSON object
    

    def get_subkey_value(self, top_level_key, subkey):
        """
        Function for getting the value of a specified subkey of a key in a JSON file.
        """
        if isinstance(self.json_obj, dict):
            if top_level_key in self.json_obj and isinstance(self.json_obj[top_level_key], dict):
                if subkey in self.json_obj[top_level_key]:
                    return self.json_obj[top_level_key][subkey]
                else:
                    return None  # Subkey not found
            else:
                return None  # Top-level key not found or is not a dictionary
        else:
            return None  # Input is not a JSON object
        


class TCP_Server:
    """
    Creates a TCP server, which awaits connection from any source and then continously recieves data on the socket.
    Takes parameters:
       Host - Server host machine IP
       Port - Server host port to communicate on
       Debug - bool print statements (defaults False)
    """

    def __init__(self, Host, Port, Debug=False):
        
        # init variables
        self.HOST = Host
        self.PORT = Port
        self.DEBUG = Debug
        self.data_string = None

        # Create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Bind the socket to a specific address and port
        self.socket.bind((self.HOST, self.PORT))

        
    def await_connection(self):
        """
        While true loop for establishing socket connections. 
        When a connection is established then recieve_data_loop function is called where data is continously recieved in a while true loop. 
        If the connection is then broken, the recieve_data_loop is broken and the flow returns here where the system awaits a new connection.
        """
        
        while True:

            # Listen for incoming connections
            self.socket.listen(1)

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ script 'EXOLIB.py' class 'serverTCP' function 'await_connection'; SYSTEM MESSAGE: Waiting for client")
            else:
                print("Waiting for client")
            
            # Wait for a client to connect (accepting any incoming connection)
            conn, addr = self.socket.accept()

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ script 'EXOLIB.py' class 'serverTCP' function 'await_connection'; SYSTEM MESSAGE: Client connected")
            else:
                print("Client connected")

            if conn:
                return conn


    def recieve_data_loop(self, connection):
        """
        While true loop handling recieving of data on the socket connection.
        If the connection is broken, then the recieved data is empty. When this happens the loop breaks.
        This function runs when a connection is established in await_connection function.
        When the loop breaks, the flow returns to await_connection.
        """

        # Receive data from the client
        data = connection.recv(1024)

        # Format the data as a string
        self.data_string = f"{str(data.decode())}"

        # If the data is empty, then the connection has been broken. Therefore no more data will arrive and the system shall attempt to reconnect.
        if self.data_string == "":
            self.await_connection()

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serverTCP' function 'recieve_data_loop'; VARIABLE 'data_string': {self.data_string}")

        return self.data_string
    

class serial2arduino:
    """
    Class for establishing serial communication between Python script and an Arduino. 
    Takes parameters:
       serial_port - the COM port connection with USB)
       baud_rate - defaults 9600
       bytesize - Number of databits transmitted in each byte of serial data.
       Debug - bool print statements (defaults False)
    """

    def __init__(self, serial_port, baud_rate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, Debug=False):

        # init variables
        self.SERIAL_PORT = serial_port
        self.BAUD_RATE = baud_rate
        self.BYTESIZE = eval(bytesize)
        self.PARITY = eval(parity)
        self.STOPBITS = eval(stopbits)
        self.DEBUG = Debug

        if self.DEBUG:
            # Find all possible open com ports
            ports = serial.tools.list_ports.comports()

            # Lists all ports gotten from above
            for port, desc, hwid in sorted(ports):
                print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function '__init__'; SYSTEM MESSAGE: List of avaible ports:\n{port}: {desc} [{hwid}]")


    def establish_connection(self):
        """
        Function for handling establishing connection between Python pyserial and Arduino.
        """
        
        # Attempt establishing a connection with the arduino until success.
        while True:
            try:
                # Open a serial connection with the Arduino
                connection = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, self.BYTESIZE, self.PARITY, self.STOPBITS, timeout=2)

                # If the DEBUG flag is raised, we print data to terminal
                if self.DEBUG:
                    print("DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'establish_connection'; SYSTEM MESSAGE: Waiting 2 (two) seconds for Arduino to initialize.")
                
                # Wait for the Arduino to initialize
                time.sleep(2)

                # If a connection has been made, the reset the input and output buffers
                # and print to terminal that the buffers has been reset
                if connection:    
                    connection.reset_input_buffer()
                    connection.reset_output_buffer()

                    print("DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'establish_connection'; SYSTEM MESSAGE: Reset buffers.")

                    return connection
            
            # Except all errors
            except Exception as e:
                if self.DEBUG:
                    print(f"ERROR @ script 'EXOLIB.py' class 'serial2arduino' function 'establish_connection'; SYSTEM MESSAGE: Failed conencting to arduino at serial port '{self.SERIAL_PORT}' with error: {e}")
                
                

    def send_data(self, connection, data, seperator, state=""):
        """
        Function for handling sending of data across serial connection established in function 'establish_connection'.
        The data that is to be send must be send as a string ending with a defined seperator matching with what is defined on the arduino.
         - Eg. 'Example_' where the underscore is the seperator. This way the arduino can interpret n amount of bytes as a whole, instead of interpreting them individually.
        """

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'send_data'; VARIABLE 'data': {data}")
    
        data = f"{state}{data}{seperator}"

        # Encodes the data string to UTF-8. The encoding can be changed by specifying which type should be used
        encoded_data = data.encode()

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'send_data'; VARIABLE 'encoded_data': {encoded_data}")

        # Attemps to send the "encoded_data" to the arduino
        # If it fails, it will print the error to the terminal
        try:
            connection.write(encoded_data)
        except Exception as e:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'send_data'; Failed to write encoded data with error: {e}")


    def receive_data(self, connection):
        """
        Function for handling reception of data from the Arduino over the established serial connection.
        """

        if self.DEBUG:
            print("DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'receive_data'; SYSTEM MESSAGE: Receiving data from Arduino.")

        # Read the data from the Arduino, and flush the input afterwards
        # such that we are ready for the next data package
        received_data = connection.readline()
        connection.flushInput()
        
        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'receive_data'; VARIABLE: 'received_data': {received_data}")
        
        # Decode the data using UTF-8, just as when we encoded the data
        decoded_data = received_data.decode()
        
        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'receive_data'; VARIABLE: 'decoded_data': {decoded_data}")
        
        # Strips the data, such that each section of the data becomes
        # independent of each other
        stripped_data = decoded_data.strip()

        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serial2arduino' function 'receive_data'; VARIABLE: 'stripped_data': {stripped_data}")

        return stripped_data
    

class RunningAverage:
    def __init__(self, buffersize, init_values):
        """
        Initialize the RunningAverage object.

        Parameters:
        - n (int): The size of the buffer for calculating the running average.
        """
        self.BUFFERSIZE = buffersize
        self.INIT_VALUES = init_values
        self.buffer = []
        self.sum = 0

        i = 0
        while i < self.BUFFERSIZE:
            self.add_data_point(self.INIT_VALUES)
            i += 1


    def add_data_point(self, data_point):
        """
        Add a new data point to the buffer and update the running sum.

        Parameters:
        - data_point: The new data point to be added to the buffer.
        """
        self.buffer.append(data_point)
        self.sum += data_point

        # If the buffer size exceeds n, remove the oldest data point
        if len(self.buffer) > self.BUFFERSIZE:
            removed_data = self.buffer.pop(0)
            self.sum -= removed_data

    def get_average(self):
        """
        Calculate and return the current running average.

        Returns:
        - float: The running average of the data points in the buffer.
        """
        if not self.buffer:
            return 0  # Return 0 if no data points are available to avoid division by zero
        return self.sum / len(self.buffer)


class LiveLFilter:
    def __init__(self, b, a):
        """
        Initialize live filter based on difference equation.

        Args:
            b (array-like): numerator coefficients obtained from scipy.
            a (array-like): denominator coefficients obtained from scipy.
        """
        # Implementation of a low pass filter, deserves further documentation - TO_DO!
        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a)-1)

    def __call__(self, x):
        return self.process(x)

    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    # This deserves further documentation as well - TO_DO!
    def _process(self, x):
        """
        Filter incoming data with standard difference equations.
        """
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y

