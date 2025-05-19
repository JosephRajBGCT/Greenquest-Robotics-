import threading
import time
import RPi.GPIO as GPIO
from w1thermsensor import W1ThermSensor

# Moisture sensor setup
MOISTURE_PIN = 17  # GPIO17
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOISTURE_PIN, GPIO.IN)

# DS18B20 setup
sensor = W1ThermSensor()

# Read temperature every 2 seconds
def read_temperature():
    while True:
        temperature_c = sensor.get_temperature()
        temperature_f = temperature_c * 9 / 5 + 32
        print(f"Temperature: {temperature_c:.2f} °C / {temperature_f:.2f} °F")
        time.sleep(2)

# Read moisture every 2 seconds
def read_moisture():
    while True:
        moisture_state = GPIO.input(MOISTURE_PIN)
        if moisture_state == GPIO.LOW:
            print("Soil Moisture: 1 (Wet)")
        else:
            print("Soil Moisture: 0 (Dry)")
        time.sleep(2)

# Start threads
try:
    temp_thread = threading.Thread(target=read_temperature)
    moisture_thread = threading.Thread(target=read_moisture)

    temp_thread.start()
    moisture_thread.start()

    temp_thread.join()
    moisture_thread.join()

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
