import serial
import csv

# Configure your serial port (e.g., COM3 for Windows, /dev/ttyUSB0 for Linux/Mac)
#serialPort = 'COM6' # Windows serial port
serialPort = '/dev/cu.usbserial-0247A5E1' # Mac serial port
baudRate = 115200 # Ensure this matches your Arduino's serial baud rate
#dataFile = 'F:\\Projects\Propulsive Lander\\Python Code\\V2.1 Data Acquisition\\Data\\Thrust Stand\\thrust_char.csv' # Windows file path
dataFile = '/Users/armenaroutiounian/Library/CloudStorage/OneDrive-Personal/Documents/Projects/Propulsive Lander/Python Code/V2.1 Data Acquisition/Data/Test Stand/test_stand_step_50.csv' # Mac file path

# Open the serial port
ser = serial.Serial(serialPort, baudRate, timeout=1)

# Open the CSV file for writing
with open(dataFile, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)

    # Write headers to the CSV file
    csv_writer.writerow(['Time [s]', 'Height [cm]', 'Velocity', 'Acceleration [m/s^2]', 'Throttle', 'Voltage [V]'])

    print("Logging data to", dataFile)
    try:
        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            if line and ',' in line:  # Ensure it contains CSV data
                # Split the line into fields and write to the CSV
                csv_writer.writerow(line.split(','))
                print("Logged:", line)
            else:
                # Optionally, print or handle non-data messages
                print("Non-data message:", line)
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        ser.close()
