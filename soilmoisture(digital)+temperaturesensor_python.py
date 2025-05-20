import time
import lgpio
import threading 
from w1thermsensor import W1ThermSensor

# Moisture sensor setup
MOISTURE_PIN = 17  # GPIO17
CHIP = 0  # Typically 0 for /dev/gpiochip0

# Open GPIO chip
handle = lgpio.gpiochip_open(CHIP)

# Claim moisture sensor pin as input
lgpio.gpio_claim_input(handle, MOISTURE_PIN)

# DS18B20 temperature sensor
sensor = W1ThermSensor()
stop_event = threading.Event()
# Read temperature every 2 seconds
def read_temperature():
    while True:
        try:
            temperature_c = sensor.get_temperature()
            temperature_f = temperature_c * 9 / 5 + 32
            print(f"Temperature: {temperature_c:.2f} C / {temperature_f:.2f} F")
        except Exception as e:
            print(f"Temperature sensor error: {e}")
        time.sleep(2)
# Read moisture every 2 seconds
def read_moisture():
    while not stop_event.is_set():
        try:
            moisture_state = lgpio.gpio_read(handle, MOISTURE_PIN)
            if moisture_state == 0:
                print("Soil Moisture: 1 (Wet)")
            else:
                print("Soil Moisture: 0 (Dry)")
        except Exception as e:
            print(f"Moisture sensor error: {e}")
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
    stop_event.set()  # Signal threads to stop
    temp_thread.join()
    moisture_thread.join()
    lgpio.gpiochip_close(handle)
    print("Cleaned up GPIO and exited.")
    
