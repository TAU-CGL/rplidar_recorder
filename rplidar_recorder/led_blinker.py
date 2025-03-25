import rclpy
from rclpy.node import Node

from gpiozero import LED
LED_PIN = 12 # Green

class LEDBlinker(Node):
    def __init__(self):
        super().__init__('led_blinker')
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.status = False
        self.led = LED(LED_PIN)

    def timer_callback(self):
        self.status = not self.status
        if self.status:
            self.led.on()
        else:
            self.led.off()

def main(args=None):
    rclpy.init(args=args)
    led_blinker = LEDBlinker()
    rclpy.spin(led_blinker)
    led_blinker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()