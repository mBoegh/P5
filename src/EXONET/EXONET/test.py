# from collections import deque

# class SignalBuffer:
#     def __init__(self, max_size=500):
#         self.buffer = deque(maxlen=max_size)

#     def add_signal(self, signal):
#         self.buffer.append(signal)

#     def get_last_signals(self, n=1):
#         return list(self.buffer)[-n:]

# # Example usage:
# signal_buffer = SignalBuffer()

# # Adding signals to the buffer
# for i in range(1, 1001):
#     signal_buffer.add_signal(i)

# # Getting the last 5 signals
# last_signals = signal_buffer.get_last_signals(1000)
# print("Last 5 signals:", last_signals)


import numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

# Generate example data (replace this with your actual incoming signals)
time = np.linspace(0, 1, 1000, endpoint=False)  # Time vector
signal = np.sin(200 * np.pi * 5 * time) + 0.2 * np.random.randn(1000)  # Example signal with noise

# Introduce very big spikes
num_spikes = 5
spike_indices = np.random.randint(0, len(signal), num_spikes)
signal[spike_indices] += 10  # Increase the amplitude of the spikes

# Apply Gaussian filter
sigma = 3  # Standard deviation of the Gaussian kernel
gaussian_filtered_signal = gaussian_filter1d(signal, sigma)

# Apply Mean filter (box filter)
window_size = 5  # Size of the moving average window
mean_filtered_signal = np.convolve(signal, np.ones(window_size)/window_size, mode='same')

# Plot the original, Gaussian-filtered, and Mean-filtered signals
plt.figure(figsize=(12, 6))
plt.plot(time, signal, label='Original Signal with Spikes', alpha=0.7)
plt.plot(time, gaussian_filtered_signal, label='Gaussian Filtered', linewidth=2)
plt.plot(time, mean_filtered_signal, label='Mean Filtered', linewidth=2)
plt.scatter(time[spike_indices], signal[spike_indices], color='red', marker='^', label='Spikes')
plt.title('Gaussian vs Mean Filter Applied to Incoming Signal with Spikes')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend()
plt.show()


# import numpy as np
# from scipy.ndimage import gaussian_filter1d
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# # Generate example data (replace this with your actual incoming signals)
# time = np.linspace(0, 1, 1000, endpoint=False)  # Time vector
# signal = np.sin(2 * np.pi * 5 * time) + 0.2 * np.random.randn(1000)  # Example signal with noise

# # Introduce very big spikes
# num_spikes = 5
# spike_indices = np.random.randint(0, len(signal), num_spikes)
# signal[spike_indices] += 10  # Increase the amplitude of the spikes

# # Parameters for the filters
# sigma = 3  # Standard deviation of the Gaussian kernel
# window_size = 21  # Size of the moving average window

# # Create subplots
# fig, ax = plt.subplots(figsize=(12, 6))
# line_original, = ax.plot(time, signal, label='Original Signal with Spikes', alpha=0.7)
# line_gaussian, = ax.plot(time, np.zeros_like(signal), label='Gaussian Filtered', linewidth=2)
# line_mean, = ax.plot(time, np.zeros_like(signal), label='Mean Filtered', linewidth=2)
# sc_spikes = ax.scatter(time[spike_indices], signal[spike_indices], color='red', marker='^', label='Spikes')

# # Function to update the plot with new data
# def update(frame):
#     new_data_point = np.sin(2 * np.pi * 5 * (time[-1] + 0.001 * frame)) + 0.2 * np.random.randn(1)  # Simulate new data
#     signal[:-1] = signal[1:]
#     signal[-1] = new_data_point[0]

#     # Apply filters to the updated signal
#     gaussian_filtered_signal = gaussian_filter1d(signal, sigma)
#     mean_filtered_signal = np.convolve(signal, np.ones(window_size) / window_size, mode='same')

#     print(gaussian_filtered_signal)
#     # Update the plot
#     line_original.set_ydata(signal)
#     line_gaussian.set_ydata(gaussian_filtered_signal)
#     line_mean.set_ydata(mean_filtered_signal)

#     # Update spike markers
#     sc_spikes.set_offsets(np.column_stack((time[spike_indices], signal[spike_indices])))

#     return line_original, line_gaussian, line_mean, sc_spikes

# # Set up the animation
# ani = FuncAnimation(fig, update, frames=200, interval=50, blit=True)

# # Display the plot
# plt.title('Real-time Data Processing with Filters')
# plt.xlabel('Time')
# plt.ylabel('Amplitude')
# plt.legend()
# plt.show()
