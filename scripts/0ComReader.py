import serial
import csv
import time
import keyboard
import re  # Regular expression for parsing the data
from datetime import datetime

# Configure the serial port (adjust COM port and baud rate as needed)
SERIAL_PORT = 'COM10'  # Change to your Arduino's port (e.g., '/dev/ttyUSB0' for Linux)
BAUD_RATE = 115200
OUTPUT_FILE = 'DataLogs/b_and_m_calibration.csv'

def read_serial_data():
    try:
        # Create a list to store data instead of writing to file immediately
        data_buffer = []

        # Open the serial port
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Reading data from {SERIAL_PORT}... Press 'q' to stop and save data.")
            
            # Updated regular expression pattern to handle millis and the data
            pattern = r"Millis:(\d+),\s*PWM:(\d+\.\d+),\s*ADC:(\d+),\s*Voltage:(\d+\.\d+)V,\s*Resistance:(\d+\.\d+)ohm,\s*LUX:(\d+\.\d+)"
            
            while True:
                try:
                    # Check if the 'q' key is pressed to stop
                    if keyboard.is_pressed("q"):
                        print("Key pressed, closing port and saving data...")
                        break
                    
                    # Read line from serial port
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Received data: {line}")  # Print to console for debugging
                        
                        # Match the line with the regular expression pattern
                        match = re.match(pattern, line)
                        if match:
                            # Extract the values from the matched groups
                            millis = int(match.group(1))  # Millis as integer
                            pwm = float(match.group(2))
                            adc = int(match.group(3))
                            voltage = float(match.group(4))
                            resistance = float(match.group(5))
                            lux = float(match.group(6))
                            
                            # Store the data in the buffer (list)
                            data_buffer.append([millis, pwm, adc, voltage, resistance, lux])
                        else:
                            print(f"Error: Data format mismatch for line: {line}")
                    
                    time.sleep(0.01)  # Small delay to reduce CPU usage
                except Exception as e:
                    print(f"Error processing line: {e}")

        # Once the loop ends (after pressing 'q'), write the collected data to the CSV file
        if data_buffer:
            with open(OUTPUT_FILE, 'w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(["Millis", "PWM", "ADC", "Voltage(V)", "Resistance(ohm)", "LUX"])  # Header
                writer.writerows(data_buffer)  # Write all the data at once
            print(f"Data saved to {OUTPUT_FILE}")
        else:
            print("No data to save.")

    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    read_serial_data()
