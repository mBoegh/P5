"""
TO DO:
 - Get settings from 'NODE_settings.py' on topic 'settings/NODE_visualizer'
 - Recieve EEG data from topic 'EEG_datastream'
 - Make UI with visualisation of detected mental command and its power (Eg. 'Lift' or 'Drop' and power 0-100)
"""

from EXONET.CLASS_json_handler import json_handler
        
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('visualizer_node')  # Set the node name


def main():
    json_file_path = ".//src//EXONET//EXONET//settings.json"

    handler = json_handler(json_file_path)

    DEBUG = handler.get_subkey_value("visualizer", "DEBUG")

    settings = ["DEBUG", DEBUG]

    rclpy.init()

    visualizer_node = MinimalPublisher()

    if DEBUG:
        print(f"DEBUG @ NODE_visualizer; Launched with settings: {settings}")

    rclpy.spin(visualizer_node)
    

if __name__ == "__main__":
    main()
    
