"""
TO DO:
 - Get settings from 'NODE_settings.py' on topic 'settings/NODE_controller'
 - Recieve EEG data from topic 'EEG_datastream'
 - Create controll system
 - Publish controll system data to topic 'Motor_signals'
"""

from EXONET.CLASS_json_handler import json_handler
        
import rclpy
from rclpy.node import Node


####################
######  MAIN  ######
####################


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('controller_node')  # Set the node name


def main():
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    handler = json_handler(json_file_path)

    DEBUG = handler.get_subkey_value("controller", "DEBUG")

    settings = ["DEBUG", DEBUG]

    rclpy.init()

    controller_node = MinimalPublisher()

    if DEBUG:
        print(f"DEBUG @ NODE_controller; Launched with settings: {settings}")

    rclpy.spin(controller_node)
    

if __name__ == "__main__":
    main()
    