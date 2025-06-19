import numpy as np
import matplotlib.pyplot as plt

# Data from your measurements
voltages_v = np.array([1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2])
current_mot1_ma = np.array([53.0, 63.5, 73.0, 82.5, 91.4, 99.8, 109.0, 117.7, 124.8, 131.9, 140.5, 152.0])
current_mot2_ma = np.array([54.0, 63.5, 73.3, 82.8, 91.2, 100.6, 109.0, 117.0, 125.0, 132.0, 142.0, 152.0])

# Convert current from mA to A
current_mot1_a = current_mot1_ma / 1000.0
current_mot2_a = current_mot2_ma / 1000.0

# Calculate resistance for each point (R = V/I) to find the average
resistance_mot1_ohm_points = voltages_v / current_mot1_a
resistance_mot2_ohm_points = voltages_v / current_mot2_a

# Calculate average resistances
avg_resistance_mot1 = np.mean(resistance_mot1_ohm_points)
avg_resistance_mot2 = np.mean(resistance_mot2_ohm_points)

print(f"Motor 1 - Average Resistance: {avg_resistance_mot1:.2f} Ohm")
print(f"Motor 2 - Average Resistance: {avg_resistance_mot2:.2f} Ohm")

# Calculate theoretical current based on average resistance (I = V / R_avg)
# We can use the same voltage points, or a smoother range for the line
voltage_line = np.linspace(min(voltages_v), max(voltages_v), 100) # For a smooth line
theoretical_current_mot1_a = voltage_line / avg_resistance_mot1
theoretical_current_mot2_a = voltage_line / avg_resistance_mot2


# Plotting
plt.figure(figsize=(10, 7)) # Adjusted figure size slightly

# Plot actual measured data points
plt.plot(voltages_v, current_mot1_a, marker='o', linestyle='', markersize=6, label='MOT 1 Measured Data')
plt.plot(voltages_v, current_mot2_a, marker='x', linestyle='', markersize=6, label='MOT 2 Measured Data')

# Plot theoretical lines based on average resistance
plt.plot(voltage_line, theoretical_current_mot1_a, linestyle='--', color='blue',
         label=f'MOT 1 - Avg R = {avg_resistance_mot1:.2f} Ω (I=V/R)')
plt.plot(voltage_line, theoretical_current_mot2_a, linestyle='--', color='orange', # Default second color
         label=f'MOT 2 - Avg R = {avg_resistance_mot2:.2f} Ω (I=V/R)')

plt.xlabel("Voltage (V)")
plt.ylabel("Current (A)")
plt.title("Motor Stall Current vs. Applied Voltage (I-V Curve)")
plt.legend(loc='best') # 'loc=best' tries to find the best place for the legend
plt.grid(True)
plt.ylim(bottom=0.04) # Current should not be negative
plt.xlim(left=0.9)   # Voltage should not be negative
plt.show()