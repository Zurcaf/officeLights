import csv
import matplotlib.pyplot as plt

# Define the CSV file to read
INPUT_FILE = 'data_log.csv'

# Low-pass filter parameters
ALPHA = 0.1  # Smoothing factor for the exponential moving average (0 < ALPHA <= 1)

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

def low_pass_filter(data, alpha=0.1):
    """
    Apply a simple exponential moving average (low-pass filter) to the data.
    
    :param data: List of data points to filter
    :param alpha: Smoothing factor (0 < alpha <= 1)
    :return: Filtered data
    """
    filtered_data = []
    filtered_value = data[0]  # Start with the first value (no filtering)
    for value in data:
        filtered_value = alpha * value + (1 - alpha) * filtered_value
        filtered_data.append(filtered_value)
    return filtered_data

def normalize_data(data):
    """
    Normalize the data to a range between 0 and 1.
    
    :param data: List of data points to normalize
    :return: Normalized data
    """
    min_value = min(data)
    max_value = max(data)
    return [(x - min_value) / (max_value - min_value) for x in data]

def plot_data(millis, pwm_values, lux_filtered_normalized):
    # Plot the PWM and normalized filtered LUX values over time
    plt.figure(figsize=(10, 6))
    
    # Plot PWM (Duty Cycle)
    plt.plot(millis, pwm_values, label='Duty Cycle (PWM)', color='blue', marker='o', linestyle='--')
    
    # Plot filtered LUX values (normalized)
    plt.plot(millis, lux_filtered_normalized, label='Filtered LUX (Normalized)', color='green', marker='x')
    
    # Formatting the plot
    plt.title('PWM and Filtered LUX Values Over Time')
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

    # Apply the low-pass filter only to LUX values
    lux_filtered = low_pass_filter(lux_values, alpha=ALPHA)

    # Normalize the filtered LUX values
    lux_filtered_normalized = normalize_data(lux_filtered)

    # Plot the data (Duty Cycle and normalized filtered LUX values)
    plot_data(millis, pwm_values, lux_filtered_normalized)
