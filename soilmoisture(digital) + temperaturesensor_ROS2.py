import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import threading
import time
import RPi.GPIO as GPIO
import os
import glob

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        # Moisture sensor setup
        self.moisture_pin = 17  # GPIO17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.moisture_pin, GPIO.IN)

        # DS18B20 setup
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]  # Assumes one sensor
        self.device_file = device_folder + '/w1_slave'

        # Publishers
        self.moisture_pub = self.create_publisher(Int32, 'soil_moisture', 10)
        self.temp_pub = self.create_publisher(Float32, 'temperature', 10)

        # Start threads
        threading.Thread(target=self.read_moisture_loop, daemon=True).start()
        threading.Thread(target=self.read_temperature_loop, daemon=True).start()

    def read_moisture_loop(self):
        while rclpy.ok():
            value = GPIO.input(self.moisture_pin)
            wet = 1 if value == GPIO.LOW else 0
            msg = Int32()
            msg.data = wet
            self.moisture_pub.publish(msg)
            self.get_logger().info(f'Soil Moisture (Wet=1): {wet}')
            time.sleep(2)

    def read_temperature_loop(self):
        while rclpy.ok():
            temp_c = self.read_temp()
            msg = Float32()
            msg.data = temp_c
            self.temp_pub.publish(msg)
            self.get_logger().info(f'Temperature: {temp_c:.2f} °C')
            time.sleep(2)

    def read_temp(self):
        try:
            with open(self.device_file, 'r') as f:
                lines = f.readlines()
            if lines[0].strip()[-3:] != 'YES':
                return -127.0
            equals_pos = lines[1].find('t=')
            if equals_pos != -1:
                temp_string = lines[1][equals_pos+2:]
                temp_c = float(temp_string) / 1000.0
                return temp_c
        except Exception as e:
            self.get_logger().error(f"Temp read error: {e}")
            return -127.0

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    rclpy.spin(node)
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
