import csv
import matplotlib.pyplot as plt

# Define the CSV file to read
INPUT_FILE = 'data_log.csv'

def read_csv_data():
    millis = []
    pwm_values = []
    lux_values = []
    
    # Read the CSV file
    with open(INPUT_FILE, 'r', newline='', encoding='utf-8') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        
        for row in reader:
            try:
                # Parse the data
                millis_value = int(row[0])  # Millis as integer
                pwm = float(row[1])
                lux = float(row[5])
                
                # Append data to respective lists
                millis.append(millis_value)
                pwm_values.append(pwm)
                lux_values.append(lux)
            except ValueError as e:
                # In case of malformed data, print an error and skip that row
                print(f"Error parsing row: {row}, skipping.")
                continue

    return millis, pwm_values, lux_values

def plot_data(millis, pwm_values, lux_values):
    # Plot the PWM and LUX values over time
    plt.figure(figsize=(10, 6))
    
    # Plot PWM (Duty Cycle)
    plt.plot(millis, pwm_values, label='Duty Cycle (PWM)', color='blue', marker='o')
    
    # Plot LUX
    plt.plot(millis, lux_values, label='LUX', color='green', marker='x')

    # Formatting the plot
    plt.title('Duty Cycle (PWM) and LUX Over Time')
    plt.xlabel('Millis (Time in ms)')
    plt.ylabel('Value')
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Read the data from the CSV file
    millis, pwm_values, lux_values = read_csv_data()

    # Plot the data
    plot_data(millis, pwm_values, lux_values)
