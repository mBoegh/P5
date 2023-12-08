import rclpy
from rclpy.node import Node
from simple_pid import PID
from  std_msgs.msg import Int16, String
from collections import deque
import numpy as np
from scipy.ndimage import gaussian_filter1d

class SignalBuffer:
    def __init__(self, max_size=500):
        self.buffer = deque(maxlen=max_size)
    def add_signal(self, signal):
        self.buffer.append(signal)
    def get_last_signals(self, n=1):
        return list(self.buffer)[-n:]

class Angle_control(Node):

    def __init__(self):
        super().__init__('sub_pot_data')
        self.publisher_ = self.create_subscription(Int16,"serial_data",self.Angle_data, 10)
        # self.velocity_motor_signals_subscription = self.create_subscription(Int16, 'Velocity_motor_signals', self.motor_signals_topic_callback, 10)
        
        self.subscriper = self.create_subscription(String, "up_down_steady_signal",self.Desired_pot_func,10)
        
        self.pid = PID(0.1,0.1,0.1)
        self.pid.output_limits = (-100,100)
        
        self.desired_pot = 0
        self.desired_angle = 0
        self.most_recent_value = 0
        
        self.signal_buffer = SignalBuffer()
        
        self.create_timer(0.01,self.control)
        
    def Angle_data(self,msg):       
        self.potentiometer = msg.data
        self.signal_buffer.add_signal(self.potentiometer)
        # Apply Gaussian filter
        sigma = 6  # Standard deviation of the Gaussian kernel
        gaussian_filtered_signal = gaussian_filter1d(self.signal_buffer.get_last_signals(500), sigma)
        # Get the most recent filtered value
        self.most_recent_value = gaussian_filtered_signal[-1]
        
    def Desired_pot_func(self,msg):
        command = msg.data

        if command == "up":
            self.desired_pot = self.most_recent_value+10            
        elif command == "down":
            self.desired_pot = self.most_recent_value-10
        else:
            self.desired_pot = self.most_recent_value
            
        
    def control(self):
        old_min = 0
        old_max = 1023
        new_min = 0
        new_max = 270

        
        self.desired_angle = np.interp(self.desired_pot, (old_min, old_max), (new_min, new_max))
        self.pid.setpoint = self.desired_angle
        
        
        lower_angle = 50
        upper_angle = 100
        
        if lower_angle < self.desired_angle < upper_angle:
            regulator = 0
            print("outside bounds!!!!!")
        else:
            regulator = self.pid(self.most_recent_value)

        print("desired angle: " , self.desired_angle)
        
        
        
        print(regulator)

def main(args=None):
    rclpy.init(args=args)
    Controllor = Angle_control()
    rclpy.spin(Controllor)
    Controllor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()