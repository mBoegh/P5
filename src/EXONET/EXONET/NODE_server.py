"""
TO DO:
 - Get settings from 'NODE_settings.py' on topic 'settings/NODE_server'
 - Publish @ class 'serverTCP' function 'recieve_data_loop'; VARIABLE 'data_string' to topic 'EEG_datastream'
"""

import socket

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
        socket_serverTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Bind the socket to a specific address and port
        socket_serverTCP.bind((self.HOST, self.PORT))

        # Functioncall await_connection with the socket as argument 
        self.await_connection(socket_serverTCP)
        
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
            
            # Wait for a client to connect (accepting any incoming connection)
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
                print(f"DEBUG @ class 'serverTCP' function 'recieve_data_loop'; VARIABLE 'data_string': {data_string}")


####################
######  MAIN  ######
####################

def main():
    print("Hello world!")

    HOST = None
    PORT = None
    DEBUG = None

    # Instance the serverTCP class
    server = serverTCP(HOST, PORT, DEBUG)

if __name__ == "__main__":
    main()
