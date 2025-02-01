import serial
import time
import csv
import matplotlib.pyplot as plt

# Replace these with the correct ports for your Arduinos
arduino1 = serial.Serial('COM3', 9600, timeout=1)  # Metro Mini
arduino2 = serial.Serial('COM6', 9600, timeout=1)  # Arduino Uno

start_time = time.time()  # Record timestamp before requesting

# Lists to store data
voltages1 = []
voltages2 = []
timestamps = []

# Output CSV file
csv_filename = "F:\Projects\Propulsive Lander\Python Code\Power Module Calibration\Data\powerModCal.csv"

try:
    print("Press Ctrl+C to stop and save data to CSV.")
    
    # Open the CSV file for writing
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Voltage1", "Voltage2"])  # Write header

        while True:
            # Request data from both Arduinos
            arduino1.write(b'R')  # Send 'R' to first Arduino
            arduino2.write(b'R')  # Send 'R' to second Arduino

            # Read data from Arduinos
            voltage1 = arduino1.readline().decode('utf-8').strip()
            voltage2 = arduino2.readline().decode('utf-8').strip()
            timestamp = time.time() - start_time # Time since request

            # Append data
            timestamps.append(timestamp)
            voltages1.append(voltage1)
            voltages2.append(voltage2)

            # Write data to CSV
            writer.writerow([timestamp, voltage1, voltage2])

            # Print for debugging
            print(f"Time: {timestamp:.4f}, Voltage1: {voltage1}, Voltage2: {voltage2}")

            # Control sampling rate
            time.sleep(1)

finally:
    # Close serial ports
    arduino1.close()
    arduino2.close()
