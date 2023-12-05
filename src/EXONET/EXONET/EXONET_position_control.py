import rclpy
from rclpy.node import Node

from simple_pid import PID
from  std_msgs.msg import Int16
from collections import deque
import numpy as np

import time

class SignalBuffer:
    def __init__(self, max_size=500):
        self.buffer = deque(maxlen=max_size)
    def add_signal(self, signal):
        self.buffer.append(signal)
    def get_last_signals(self, n=1):
        return list(self.buffer)[-n:]



class Angle_control(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_subscription(Int16,"Feedback_raw_joint_data",self.Angle_control, 10)
        self.velocity_motor_signals_subscription = self.create_subscription(Int16, 'Velocity_motor_signals', self.motor_signals_topic_callback, 10)
        
        self.pid = PID(1,1,0)
        self.pid.output_limits = (-100,100)
        
        self.signal_buffer = SignalBuffer()
        
        



    def Angle_data(self,msg):       
        self.potentiometer = msg.data
        self.signal_buffer.add_signal(self.potentiometer)
        
        pot = np.average(self.signal_buffer.get_last_signals(20))
        
        
        
        
        
    def Desired_angle(self,t):
        angle = t+1

    def control(self):
        self.pid.setpoint(self.Desired_angle)

        
        
        
        


def main(args=None):
    rclpy.init(args=args)
    Controllor = Angle_control()
    rclpy.spin(Controllor)
    Controllor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()