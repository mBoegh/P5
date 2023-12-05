from collections import deque

class SignalBuffer:
    def __init__(self, max_size=500):
        self.buffer = deque(maxlen=max_size)

    def add_signal(self, signal):
        self.buffer.append(signal)

    def get_last_signals(self, n=1):
        return list(self.buffer)[-n:]

# Example usage:
signal_buffer = SignalBuffer()

# Adding signals to the buffer
for i in range(1, 1001):
    signal_buffer.add_signal(i)

# Getting the last 5 signals
last_signals = signal_buffer.get_last_signals(1000)
print("Last 5 signals:", last_signals)
