import csv
import matplotlib.pyplot as plt
import numpy as np

# Define the CSV file to read
INPUT_FILE = 'data_log.csv'

# Low-pass filter parameters
ALPHA = 0.6  # Smoothing factor for the exponential moving average (0 < ALPHA <= 1)

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

def remove_outliers(data):
    """
    Remove outliers using the Interquartile Range (IQR) method.
    
    :param data: List of data points to remove outliers from.
    :return: Data without outliers and indices of outliers.
    """
    # Convert data to a numpy array for easy manipulation
    data = np.array(data)
    
    # Calculate Q1, Q3 and IQR
    Q1 = np.percentile(data, 25)
    Q3 = np.percentile(data, 75)
    IQR = Q3 - Q1
    
    # Define bounds for outliers
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    
    # Identify outliers
    outliers = (data < lower_bound) | (data > upper_bound)
    
    # Remove outliers from data
    filtered_data = [x for x, outlier in zip(data, outliers) if not outlier]
    
    return filtered_data, outliers

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

def plot_data(millis, pwm_values, lux_values, lux_filtered, outlier_indices):
    # Plot the PWM and LUX values over time
    plt.figure(figsize=(10, 6))
    
    # Plot PWM (Duty Cycle) without any filtering
    plt.plot(millis, pwm_values, label='Raw PWM', color='blue', marker='o', linestyle='--')
    
    # Plot LUX with raw and filtered data
    plt.plot(millis, lux_values, label='Raw LUX', color='green', marker='x', linestyle='--')
    plt.plot(millis, lux_filtered, label='Filtered LUX (Low-pass)', color='green', marker='x')

    # Highlight the outliers in red
    outlier_millis = np.array(millis)[outlier_indices]
    outlier_lux = np.array(lux_values)[outlier_indices]
    plt.scatter(outlier_millis, outlier_lux, color='red', label='Outliers', zorder=5)

    # Formatting the plot
    plt.title('PWM and LUX Over Time with Outlier Detection and Low-pass Filter for LUX')
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

    # Remove outliers from LUX values and get outlier indices
    lux_cleaned, outlier_indices = remove_outliers(lux_values)

    # Apply the low-pass filter only to cleaned LUX values
    lux_filtered = low_pass_filter(lux_cleaned, alpha=ALPHA)

    # Plot the data (PWM without filter, LUX with and without filter, outliers in red)
    plot_data(millis, pwm_values, lux_values, lux_filtered, outlier_indices)
