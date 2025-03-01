import math
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

class LuxMeter:
    def __init__(self, vcc, r_fixed, adc_range, m, b):
        self.vcc = vcc
        self.r_fixed = r_fixed
        self.adc_range = adc_range
        self.m = m
        self.b = b
    
    def adc_to_voltage(self, adc_value):
        return (adc_value * self.vcc) / (self.adc_range - 1)
    
    def voltage_to_resistance(self, v_adc):
        if v_adc <= 0:
            return float('inf')  # Avoid division by zero
        return self.r_fixed * ((self.vcc / v_adc) - 1)
    
    def voltage_to_lux(self, v_adc):
        r_ldr = self.voltage_to_resistance(v_adc)
        if r_ldr <= 0 or math.isinf(r_ldr):
            return 0
        log_lux = (math.log10(r_ldr) - self.b) / self.m
        return 10 ** log_lux
    
    def adc_to_lux(self, adc_value):
        v_adc = self.adc_to_voltage(adc_value)
        return self.voltage_to_lux(v_adc)

# Load data from CSV
df = pd.read_csv("data_log.csv")

# Sort data by time (Millis)
df = df.sort_values(by=["Millis"])

# Define parameter ranges
m_values = np.linspace(-1.3, -0.6, 20)  # Test 10 values of m
b_values = np.linspace(5.976, 6.277, 20)  # Test 10 values of b
vcc = 3.3
r_fixed = 10000
adc_range = 4096

# Store results: (m, b, SSE, steady_state_df)
results = []

# Iterate over all combinations of m and b
for m in m_values:
    for b in b_values:
        # Initialize LuxMeter with current m and b
        lux_meter = LuxMeter(vcc=vcc, r_fixed=r_fixed, adc_range=adc_range, m=m, b=b)
        
        # Convert ADC values to Lux
        df["LUX_Calculated"] = df["ADC"].apply(lux_meter.adc_to_lux)
        
        # Calculate steady-state Lux for each DutyCycle
        steady_state_lux = []
        for duty_cycle in sorted(df['DutyCycle'].unique()):
            duty_cycle_data = df[df['DutyCycle'] == duty_cycle]
            steady_state_data = duty_cycle_data.tail(int(len(duty_cycle_data) * 0.1))
            avg_lux = steady_state_data['LUX_Calculated'].mean()
            steady_state_lux.append((duty_cycle, avg_lux))
        
        # Convert to DataFrame
        steady_state_df = pd.DataFrame(steady_state_lux, columns=['DutyCycle', 'Avg_LUX'])
        
        # Perform linear regression
        X = steady_state_df['DutyCycle'].values.reshape(-1, 1)
        y = steady_state_df['Avg_LUX'].values
        model = LinearRegression()
        model.fit(X, y)
        y_pred = model.predict(X)
        
        # Calculate SSE (Sum of Squared Errors)
        sse = np.sum((y - y_pred) ** 2)
        results.append((m, b, sse, steady_state_df, model))

# Find the best combination (lowest SSE)
best_result = min(results, key=lambda x: x[2])  # Minimize SSE
best_m, best_b, best_sse, best_steady_state_df, best_model = best_result

# Print the best parameters
print(f"Best m: {best_m:.3f}, Best b: {best_b:.3f}, SSE: {best_sse:.4f}")

# Generate points for the regression line
X_line = np.linspace(min(best_steady_state_df['DutyCycle']), max(best_steady_state_df['DutyCycle']), 100).reshape(-1, 1)
y_line = best_model.predict(X_line)

# Plot Lux vs. Duty Cycle for the best combination with regression line
plt.figure(figsize=(8, 5))
plt.plot(best_steady_state_df['DutyCycle'], best_steady_state_df['Avg_LUX'], marker='o', linestyle='-', color='green', label=f'Data (m={best_m:.3f}, b={best_b:.3f})')
plt.plot(X_line, y_line, linestyle='--', color='blue', label=f'Linear Fit (SSE={best_sse:.4f})')
plt.xlabel("Duty Cycle")
plt.ylabel("Average Lux")
plt.title("Average Lux vs. Duty Cycle with Linear Regression (Best Fit using SSE)")
plt.legend()
plt.grid(True)
plt.show()