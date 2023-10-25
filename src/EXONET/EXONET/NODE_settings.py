####################
######  MAIN  ######
####################

"""
TO DO:
 - Publish settings nodes on their respective topics of format 'settings/*node_name*'
"""

def main():

    print("Hello world!")

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

if __name__ == "__main__":
    main()