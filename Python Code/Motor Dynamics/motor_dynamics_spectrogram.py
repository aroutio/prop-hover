import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import spectrogram

# Load the audio file
filename = "F:\Projects\Propulsive Lander\Data\Motor Dynamics\motor dynamics 10-100 long.wav"  # Replace with your audio file
sample_rate, audio_data = wavfile.read(filename)

# If stereo, take one channel
if audio_data.ndim > 1:
    audio_data = audio_data[:, 0]

# Normalize audio data
audio_data = audio_data / np.max(np.abs(audio_data))

# High-resolution spectrogram parameters
nperseg = int(8192/2)  # Larger segment size for better frequency resolution
noverlap = int(6144/2)  # 75% overlap
window = 'hann'  # Hann window to reduce spectral leakage

# Compute the spectrogram with zero padding for improved frequency resolution
nfft = 16384  # Increase the number of points in FFT (zero-padding)

frequencies, times, spectrogram_data = spectrogram(audio_data, fs=sample_rate, nperseg=nperseg, noverlap=noverlap, window=window, nfft=nfft)

# Focus on a frequency range (optional)
min_freq = 50
max_freq = 300
freq_mask = (frequencies >= min_freq) & (frequencies <= max_freq)
frequencies = frequencies[freq_mask]
spectrogram_data = spectrogram_data[freq_mask, :]

# Plot the high-resolution spectrogram
plt.figure(figsize=(14, 7))
plt.pcolormesh(times, frequencies, 10 * np.log10(spectrogram_data), shading='gouraud', cmap='viridis')
plt.title(f"High-Resolution Spectrogram (Focused on {min_freq}-{max_freq} Hz)")
plt.ylabel("Frequency (Hz)")
plt.xlabel("Time (s)")
plt.colorbar(label="Power (dB)")
plt.tight_layout()
plt.show()