import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class ArrowKeyPublisher(Node):

    def __init__(self):
        super().__init__('arrow_key_publisher')
        self.publisher_ = self.create_publisher(String, 'up_down_steady_signal', 10)
        self.timer_ = self.create_timer(0.01, self.publish_arrow_signal)
        self.key_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_listener.start()

    def on_press(self, key):
        if key == keyboard.Key.up:
            self.publish_message("up")
        elif key == keyboard.Key.down:
            self.publish_message("down")

    def on_release(self, key):
        if key == keyboard.Key.up or key == keyboard.Key.down:
            self.publish_message("stay")

    def publish_arrow_signal(self):
        # This function is still necessary for the timer
        pass

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    arrow_key_publisher = ArrowKeyPublisher()

    try:
        rclpy.spin(arrow_key_publisher)
    except KeyboardInterrupt:
        pass

    arrow_key_publisher.key_listener.stop()
    arrow_key_publisher.key_listener.join()
    arrow_key_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
