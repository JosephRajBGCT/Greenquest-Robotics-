import threading
import time

# For ADS1115 (Soil Moisture)
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

# For DS18B20 (Temperature)
from w1thermsensor import W1ThermSensor

class SensorReader:
    def __init__(self):
        # Initialize I2C and ADS1115
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS1115(i2c)
        self.channel = AnalogIn(self.ads, ADS1115.P0)  # A0 for soil moisture sensor

        # Initialize DS18B20
        self.temp_sensor = W1ThermSensor()

        # Start parallel threads
        threading.Thread(target=self.read_moisture, daemon=True).start()
        threading.Thread(target=self.read_temperature, daemon=True).start()

    def read_moisture(self):
        while True:
            voltage = self.channel.voltage
            moisture_percent = (1 - voltage / 3.3) * 100  # 0V = wet, 3.3V = dry
            moisture_percent = max(0.0, min(100.0, moisture_percent))  # Clamp value

            print(f"[Soil Moisture] Voltage: {voltage:.2f} V, Moisture: {moisture_percent:.2f} %")
            time.sleep(2)

    def read_temperature(self):
        while True:
            temperature_c = self.temp_sensor.get_temperature()
            print(f"[Temperature] DS18B20: {temperature_c:.2f} °C")
            time.sleep(2)

if __name__ == "__main__":
    print("Starting sensor readings (Soil Moisture + Temperature)...")
    reader = SensorReader()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping sensor readings.")
