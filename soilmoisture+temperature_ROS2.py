import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import threading
import time

# For ADS1115 (Soil Moisture)
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

# For DS18B20 (Temperature)
from w1thermsensor import W1ThermSensor

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader_node')

        # Publishers
        self.moisture_pub = self.create_publisher(Float32, 'soil_moisture', 10)
        self.temp_pub = self.create_publisher(Float32, 'temperature_celsius', 10)

        # I2C for ADS1115
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS1115(i2c)
        self.channel = AnalogIn(self.ads, ADS1115.P0)  # A0 input

        # DS18B20 Sensor
        self.temp_sensor = W1ThermSensor()

        # Start threads
        threading.Thread(target=self.read_moisture, daemon=True).start()
        threading.Thread(target=self.read_temperature, daemon=True).start()

    def read_moisture(self):
        while rclpy.ok():
            voltage = self.channel.voltage
            # Calibrate this mapping based on your sensor's dry/wet voltage range
            # Assuming 0V (wet) to 3.3V (dry)
            moisture_percent = (1 - voltage / 3.3) * 100
            moisture_percent = max(0.0, min(100.0, moisture_percent))  # Clamp

            msg = Float32()
            msg.data = round(moisture_percent, 2)
            self.moisture_pub.publish(msg)
            self.get_logger().info(f'Soil Moisture: {msg.data} %')

            time.sleep(2)

    def read_temperature(self):
        while rclpy.ok():
            temperature_c = self.temp_sensor.get_temperature()
            msg = Float32()
            msg.data = round(temperature_c, 2)
            self.temp_pub.publish(msg)
            self.get_logger().info(f'Temperature: {msg.data} °C')
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

