import rclpy
from rclpy.node import Node

from gpiozero import LED
LED_PIN = 16 # Red

class RedLED(Node):
    def __init__(self):
        super().__init__('red_led')
        self.led = LED(LED_PIN)
        self.led.on()

def main(args=None):
    rclpy.init(args=args)
    red_led = RedLED()
    rclpy.spin(red_led)
    red_led.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()