import sys
import os
import serial
import time
import numpy as np

# Connect to Arduino via serial
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Ensure the correct port

def send_to_arduino(meal_times, Dg, t, y):
    data = ",".join(map(str, meal_times + Dg + [t] + y))
    arduino.write((data + "\n").encode())

def read_from_arduino():
    try:
        line = arduino.readline().decode().strip()
        glucose, insulin = map(float, line.split(","))
        return glucose, insulin
    except:
        return None, None


def pid(setpoint, G, prev_error, integral, Kp, Ki, Kd, sample_rate):
    error = G - setpoint
    integral += error * sample_rate
    derivative = (error - prev_error) / sample_rate
    control_signal = Kp * error + Ki * integral + Kd * derivative
    control_signal = max(control_signal, 0)  # Ensure control signal is non-negative
    return control_signal, error, integral


def run_simulation(c, meal_times, Dg, t_points, setpoint, Kp, Ki, Kd):
    prev_error = 0
    integral = 0
    sample_rate = c.h

    total_insulin_administered = 0

    y = [c.G, c.I, c.X]

    for t in t_points:
        # Send the current state to Arduino
        send_to_arduino(meal_times, Dg, t, y)

        # Receive the updated glucose and insulin from Arduino
        G, I = read_from_arduino()
        if G is None: continue

        # Run PID control on Jetson
        control_signal, prev_error, integral = pid(setpoint, G, prev_error, integral, Kp, Ki, Kd, sample_rate)
        total_insulin_administered += control_signal * sample_rate

    insulin_in_grams = total_insulin_administered * 3.47e-11  # Conversion to grams
    print(f"Total Insulin in micro-units: {total_insulin_administered:.2f}")
    print(f"Total Insulin in grams: {insulin_in_grams:.10f}")


if __name__ == '__main__':
    class Constant:
        MAX_TIME = 1800
        h = 1
        G = 110
        I = 10
        X = 0

    c = Constant()
    t_points = np.arange(0, c.MAX_TIME, c.h)

    setpoint = 130
    Kp = 0.5
    Ki = Kp / 100
    Kd = Kp * 0.1

    meal_times = [100, 400, 1200]
    Dg = [60, 45, 50]

    run_simulation(c, meal_times, Dg, t_points, setpoint, Kp, Ki, Kd)
