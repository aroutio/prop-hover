import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile

# Load the audio file
filename = "F:\Projects\Propulsive Lander\Data\Motor Dynamics\motor dynamics 15.wav"  # Replace with your audio file
sample_rate, audio_data = wavfile.read(filename)

# If stereo, take one channel
if audio_data.ndim > 1:
    audio_data = audio_data[:, 0]

# Normalize audio data
audio_data = audio_data / np.max(np.abs(audio_data))

# Perform FFT
n = len(audio_data)
fft_data = np.fft.fft(audio_data)
frequencies = np.fft.fftfreq(n, d=1/sample_rate)

# Take the magnitude of FFT and focus on positive frequencies
fft_magnitude = np.abs(fft_data[:n // 2])
positive_frequencies = frequencies[:n // 2]

# Plot the frequency spectrum
plt.figure(figsize=(12, 6))
plt.plot(positive_frequencies, fft_magnitude)
plt.title("Frequency Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.grid()
plt.show()
