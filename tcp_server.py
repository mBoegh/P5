import socket

class serverTCP:
    """
    Creates a TCP server, which awaits connection from any source and then continously recieves data on the socket.
    """

    def __init__(self, Host, Port, Debug=True):
        
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
                print("Waiting for client")
            
            # Wait for a client to connect
            conn, addr = socket.accept()

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
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

            # Process the data
            if self.DEBUG:
                print(data_string)


def main():

    ## Define server host ip
    HOST = "172.26.50.145"

    ## Define server port
    PORT = 20000

    ## Variable for enabling/disabling print statements
    DEBUG = True

    # Instance the serverTCP class
    server = serverTCP(HOST, PORT, DEBUG)

if __name__ == '__main__':
    main()
