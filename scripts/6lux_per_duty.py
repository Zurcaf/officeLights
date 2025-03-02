import math
import pandas as pd
import matplotlib.pyplot as plt

class LuxMeter:
    def __init__(self, vcc, r_fixed, adc_range, m=-0.8, b=5.976):
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
df = pd.read_csv("data_log_A.csv")

# Sort data by time (Millis)
df = df.sort_values(by=["Millis"])

# Initialize LuxMeter with appropriate parameters
lux_meter = LuxMeter(vcc=3.3, r_fixed=10000, adc_range=4096, m=-0.8, b=5.976)

# Convert ADC values to Lux
df["LUX_Calculated"] = df["ADC"].apply(lux_meter.adc_to_lux)

# Verify if calculated Lux matches received Lux for the same time values
df_grouped = df.groupby("Millis").agg({"LUX": "mean", "LUX_Calculated": "mean"}).reset_index()
df_grouped["Match"] = df_grouped["LUX_Calculated"].round(2) == df_grouped["LUX"].round(2)

# Print mismatched values if any
mismatches = df_grouped[df_grouped["Match"] == False]
if not mismatches.empty:
    print("Mismatched Lux values:")
    print(mismatches[["Millis", "LUX", "LUX_Calculated"]])

# Plot Lux vs. Time
plt.figure(figsize=(8, 5))
plt.plot(df_grouped["Millis"], df_grouped["LUX_Calculated"], marker='o', linestyle='-', label="Calculated Lux")
plt.scatter(df_grouped["Millis"], df_grouped["LUX"], color='red', label="Received Lux", zorder=3)
plt.xlabel("Time (ms)")
plt.ylabel("Lux")
plt.title("Lux vs. Time")
plt.legend()
plt.grid(True)
plt.show()


# Assuming a 'DutyCycle' column exists in the CSV for each data point

# Group data by DutyCycle and select the last portion for steady state
# We will select the last 10% of the data points per duty cycle as the steady state
steady_state_lux = []

# Iterate through unique duty cycle values (from 0.1 to 1)
for duty_cycle in sorted(df['DutyCycle'].unique()):
    # Get the subset of data for the given duty cycle
    duty_cycle_data = df[df['DutyCycle'] == duty_cycle]
    
    # Select the last 10% of the data as steady state
    steady_state_data = duty_cycle_data.tail(int(len(duty_cycle_data) * 0.1))
    
    # Calculate the average Lux for the steady state
    avg_lux = steady_state_data['LUX_Calculated'].mean()
    steady_state_lux.append((duty_cycle, avg_lux))

# Convert the list to a DataFrame for easy plotting
steady_state_df = pd.DataFrame(steady_state_lux, columns=['DutyCycle', 'Avg_LUX'])

# Plot Lux vs. Duty Cycle
plt.figure(figsize=(8, 5))
plt.plot(steady_state_df['DutyCycle'], steady_state_df['Avg_LUX'], marker='o', linestyle='-', color='green')
plt.xlabel("Duty Cycle")
plt.ylabel("Average Lux")
plt.title("Average Lux vs. Duty Cycle")
plt.grid(True)
plt.show()
