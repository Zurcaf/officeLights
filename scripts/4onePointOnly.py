import csv
import matplotlib.pyplot as plt
import numpy as np

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

def get_steady_state_lux(pwm_values, lux_values):
    """
    Compute the mean LUX values for each unique PWM value (steady-state for each PWM value).
    
    :param pwm_values: List of PWM duty cycle values
    :param lux_values: List of LUX values
    :return: Lists of unique PWM values and their corresponding steady-state LUX mean values
    """
    pwm_unique = np.unique(pwm_values)  # Get unique PWM values
    steady_state_lux = []
    
    for pwm in pwm_unique:
        # Find all indices where the PWM value is equal to the current unique PWM
        indices = [i for i, pwm_val in enumerate(pwm_values) if pwm_val == pwm]
        
        # Take the mean of the LUX values at those indices
        lux_mean = np.mean([lux_values[i] for i in indices])
        steady_state_lux.append(lux_mean)
    
    return pwm_unique, steady_state_lux

def linear_regression(pwm_values, lux_values):
    """
    Perform linear regression and compute the Sum of Squared Errors (SSE).
    
    :param pwm_values: List of unique PWM values (X-axis)
    :param lux_values: List of steady-state normalized LUX values (Y-axis)
    :return: Regression coefficients (slope and intercept), predicted values, and SSE
    """
    # Fit a linear model (y = mx + b)
    slope, intercept = np.polyfit(pwm_values, lux_values, 1)
    
    # Compute the predicted lux values
    lux_predicted = [slope * pwm + intercept for pwm in pwm_values]

    # Compute SSE (Sum of Squared Errors)
    sse = sum((lux_actual - lux_pred) ** 2 for lux_actual, lux_pred in zip(lux_values, lux_predicted))

    return slope, intercept, lux_predicted, sse

def plot_data(pwm_values, steady_state_lux, lux_predicted, slope, intercept, sse):
    """
    Plot the steady-state LUX values along with the linear regression line.
    
    :param pwm_values: List of unique PWM values
    :param steady_state_lux: Corresponding normalized LUX values
    :param lux_predicted: Predicted LUX values from linear regression
    :param slope: Regression slope
    :param intercept: Regression intercept
    :param sse: Sum of Squared Errors
    """
    plt.figure(figsize=(10, 6))
    
    # Scatter plot for the steady-state LUX values
    plt.scatter(pwm_values, steady_state_lux, label='Steady-state LUX (Normalized)', color='green', marker='x')
    
    # Plot the linear regression line
    plt.plot(pwm_values, lux_predicted, label=f'Linear Fit: y = {slope:.4f}x + {intercept:.4f}', color='red', linestyle='-')
    
    # Formatting the plot
    plt.title(f'Duty Cycle (PWM) vs Steady-state Normalized LUX\nSSE = {sse:.4f}')
    plt.xlabel('Duty Cycle (PWM)')
    plt.ylabel('Normalized LUX')
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

    # Compute the steady-state LUX values for each PWM value
    pwm_unique, steady_state_lux = get_steady_state_lux(pwm_values, lux_filtered_normalized)

    # Perform linear regression
    slope, intercept, lux_predicted, sse = linear_regression(pwm_unique, steady_state_lux)

    # Plot the data with regression line
    plot_data(pwm_unique, steady_state_lux, lux_predicted, slope, intercept, sse)

    # Print the regression equation and SSE
    print(f"Linear Regression Equation: y = {slope:.4f}x + {intercept:.4f}")
    print(f"Sum of Squared Errors (SSE): {sse:.4f}")
