import numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

# Generate example data (replace this with your actual incoming signals)
time = np.linspace(0, 1, 1000, endpoint=False)  # Time vector
signal = np.sin(2 * np.pi * 5 * time) + 0.2 * np.random.randn(1000)  # Example signal

# Apply Gaussian filter
sigma = 3  # Standard deviation of the Gaussian kernel
gaussian_filtered_signal = gaussian_filter1d(signal, sigma)

# Get the most recent filtered value
most_recent_value = gaussian_filtered_signal[-1]

# Print or use the most recent value as needed
print("Most recent filtered value:", most_recent_value)

# Plot the original and filtered signals
plt.figure(figsize=(10, 6))
plt.plot(time, signal, label='Original Signal', alpha=0.7)
plt.plot(time, gaussian_filtered_signal, label='Gaussian Filtered', linewidth=2)
plt.scatter(time[-1], most_recent_value, color='red', marker='o', label='Most Recent Value')
plt.title('Gaussian Filter Applied to Incoming Signal')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
