import socket
import serial
import time

class serverTCP:
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

        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Bind the socket to a specific address and port
        s.bind((self.HOST, self.PORT))

        # Functioncall await_connection with the socket as argument 
        self.await_connection(s)
        
    def await_connection(self, socket):
        """
        While true loop for establishing socket connections. 
        When a connection is established then recieve_data_loop function is called where data is continously recieved in a while true loop. 
        If the connection is then broken, the recieve_data_loop is broken and the flow returns here where the system awaits a new connection.
        """
        
        while True:

            # Listen for incoming connections
            socket.listen(1)

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ class 'serverTCP' function 'await_connection'; SYSTEM MESSAGE: Waiting for client")
            else:
                print("Waiting for client")
            
            # Wait for a client to connect
            conn, addr = socket.accept()

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ class 'serverTCP' function 'await_connection'; SYSTEM MESSAGE: Client connected")
            else:
                print("Client connected")

            # If a connection is established, then we move to revcieve data loop
            if conn:
                self.recieve_data_loop(conn)


    def recieve_data_loop(self, connection):
        """
        While true loop handling recieving of data on the socket connection.
        If the connection is broken, then the recieved data is empty. When this happens the loop breaks.
        This function runs when a connection is established in await_connection function.
        When the loop breaks, the flow returns to await_connection.
        """

        while True:
            # Receive data from the client
            data = connection.recv(1024)

            # Format the data as a string
            data_string = f"{str(data.decode())}"

            # If the data is empty, then the connection has been broken. Therefore no more data will arrive and the loop shall break.
            if data_string == "":
                break

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ class 'serverTCP' function 'recieve_data_loop'; Variable 'data_string': {data_string}")


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
        arduino = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, self.TIMEOUT)

        # Wait for the Arduino to initialize
        time.sleep(2)

    def send_data(self, data):
        """
        Function for handling sending of data across serial connection established in function 'establish_connection'.
        """

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ class 'serial2arduino' function 'Send_data'; Variable 'data': {data}")
    
        # Encode data as encoded_data
        encoded_data = data.encode()

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ class 'serial2arduino' function 'Send_data'; Variable 'encoded_data': {encoded_data}")

        # Send encoded_data Arduino
        self.write(encoded_data)

def main():

    ## Variable for enabling/disabling print statements
    DEBUG = True

    ## Define tcp server host ip
    HOST = "172.26.50.145"

    ## Define tcp server port
    PORT = 20000

    ## Define serial port to communicate with arduino across
    SERIAL_PORT = "COM3"

    ## Define arduino baud rate
    BAUD_RATE = 9600

    ## Define amount of seconds before arduino connection attempt is abandonded
    TIMEOUT = 1

    # Instance the serial2arduino class
    arduino = serial2arduino(SERIAL_PORT, BAUD_RATE, TIMEOUT, DEBUG)

    # Instance the serverTCP class
    server = serverTCP(HOST, PORT, DEBUG)

if __name__ == '__main__':
    main()
