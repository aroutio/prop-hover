import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
import pywt

# Load the audio file
filename = "F:\Projects\Propulsive Lander\Data\Motor Dynamics\motor dynamics 15-50 shorter.wav"  # Replace with your audio file
sample_rate, audio_data = wavfile.read(filename)

# If stereo, take one channel
if audio_data.ndim > 1:
    audio_data = audio_data[:, 0]

# Normalize audio data
audio_data = audio_data / np.max(np.abs(audio_data))

# Define parameters for the Wavelet Transform
min_freq = 50  # Minimum frequency of interest
max_freq = 300  # Maximum frequency of interest
num_frequencies = 100  # Number of frequency bands
scales = np.linspace(sample_rate / max_freq, sample_rate / min_freq, num_frequencies)

# Perform Continuous Wavelet Transform (CWT) using the Morlet wavelet
wavelet = 'cmor'  # Complex Morlet wavelet
coefficients, frequencies = pywt.cwt(audio_data, scales, wavelet, sampling_period=1/sample_rate)

# Plot the Wavelet Transform
plt.figure(figsize=(12, 6))
plt.imshow(
    np.abs(coefficients), 
    extent=[0, len(audio_data) / sample_rate, min_freq, max_freq],
    cmap='viridis',
    aspect='auto',
    origin='lower'
)
plt.colorbar(label="Amplitude")
plt.title(f"Wavelet Transform (Focused on {min_freq}-{max_freq} Hz)")
plt.ylabel("Frequency (Hz)")
plt.xlabel("Time (s)")
plt.tight_layout()
plt.show()
