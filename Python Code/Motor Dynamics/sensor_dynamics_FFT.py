import numpy as np
import matplotlib.pyplot as plt
#from scipy import signal

# Load data from a file (assume a single column of distance values)
# Replace 'data.csv' with your file name
data = np.loadtxt("F:\Projects\Propulsive Lander\Data\Sensor Dynamics\distance_sensor_kp07_ki0_kd01.csv")
time = np.loadtxt("F:\Projects\Propulsive Lander\Data\Sensor Dynamics\distance_sensor_kp07_ki0_kd01_time.csv")

filteredData = [0]
alpha = 0.8

for i in range(0, len(data) - 1):
    filteredData.append(alpha * data[i] + (1 - alpha) * filteredData[i])

print(len(time))
print(len(data))
print(len(filteredData))

plt.figure(figsize=(10, 6))
plt.plot(time, data)
plt.plot(time, filteredData)
plt.grid()
plt.show()


# Sampling frequency (samples per second)
#sampling_frequency = 1000 / 50  # Adjust based on your Arduino sampling rate

# Perform FFT
#fft_result = np.fft.fft(data)
#frequencies = np.fft.fftfreq(len(data), d=1/sampling_frequency)

# Only consider positive frequencies (FFT result is symmetric)
#positive_freq_indices = np.where(frequencies > 0)
#frequencies = frequencies[positive_freq_indices]
#fft_magnitude = np.abs(fft_result[positive_freq_indices])

# Plot the results
#plt.figure(figsize=(10, 6))
#plt.plot(frequencies, fft_magnitude)
#plt.title('Frequency Spectrum')
#plt.xlabel('Frequency (Hz)')
#plt.ylabel('Amplitude')
#plt.grid()
#plt.show()

# Use the code below of highly periodic signal noise reduction??
# Continous transfer function
#w0 = 2 * np.pi * 0.1
#num = w0
#den = [1 , w0]
#lowPass = signal.TransferFunction(num, den)
#print(lowPass)

# Discrete transfer function
#dt = 1 / sampling_frequency
#discreteLowPass = lowPass.to_discrete(dt, method = 'gbt', alpha = 0.5)
#print(discreteLowPass)

# Difference equation coefficients
#b = discreteLowPass.num
#a = -discreteLowPass.den
#print("Filter coefficients b_i: " + str(b))
#print("Filter coefficients a_i: " + str(a[1:]))

#yn = []
#yn_1 = 0
#xn_1 = 0

#for i in range(0, len(data)):
    #yn.append(a[0] * yn_1 + b[0] * data[i] + b[1] * xn_1)
    #yn_1 = yn[i]
    #xn_1 = data[i]

#plt.figure(figsize=(10, 6))
#plt.plot(time, data)
#plt.plot(time, yn)
#plt.grid()
#plt.show()

# Perform FFT
#fft_result_filtered = np.fft.fft(yn)
#frequencies_filtered = np.fft.fftfreq(len(yn), d=1/sampling_frequency)

# Only consider positive frequencies (FFT result is symmetric)
#positive_freq_indices_filtered = np.where(frequencies_filtered > 0)
#frequencies_filtered = frequencies_filtered[positive_freq_indices_filtered]
#fft_magnitude_filtered = np.abs(fft_result_filtered[positive_freq_indices_filtered])

# Plot the results
#plt.figure(figsize=(10, 6))
#plt.plot(frequencies_filtered, fft_magnitude_filtered)
#plt.title('Frequency Spectrum')
#plt.xlabel('Frequency (Hz)')
#plt.ylabel('Amplitude')
#plt.grid()
#plt.show()