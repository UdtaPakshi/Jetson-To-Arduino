import serial
import time

# Initialize serial connection (adjust port as needed, usually ttyUSB0 or ttyACM0)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Give some time to establish the connection
time.sleep(2)

while True:
    # Check if data is available
    if ser.in_waiting > 0:
        # Read the incoming data
        data = ser.readline().decode('utf-8').rstrip()
        print(f"Received: {data}")
