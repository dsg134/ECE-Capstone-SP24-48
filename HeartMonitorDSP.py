from scipy.fft import fft, ifft
import numpy as np
import matplotlib.pyplot as plt

# DSP Code for Heartbeat Signal Reconstruction
# Rutgers ECE Capstone SP24-48

class HeartMonitorDSP:

    # Constructor to allocate variables
    def __init__(self):
        # Define heart data information
        self.sample_properties = []  # Sample size and spacing
        self.heart_samples = []  # Heart sample vector
        self.spacing = None  # Sample spacing

        # Define DSP Data
        self.FFT = None
        self.Filtered_FFT = None

    # Store heart data information
    def Init_heart_data(self, heart_data, spacing):
        # Ensure vectors are initially empty
        self.sample_properties.clear()
        self.heart_samples.clear()

        # Store data properties into vectors
        # sample_properties[0] = length of data
        # sample_properties[1] = sample spacing
        self.sample_properties.append(len(heart_data))
        self.sample_properties.append(spacing)

        # Store heart data
        self.heart_samples.append(heart_data)

    # Takes the FFT of heartbeat data from receiver
    def FFT_Heart_Signal(self, Input_vector):
        FFT_vector = np.array(Input_vector)
        self.FFT = fft(FFT_vector)

        return self.FFT

    # Filter out unwanted high frequency noise
    def Lowpass_Filter(self, FFT_vector):
        cutoff = 14 # Hz
        frequency_coefficients = np.fft.fftfreq(len(FFT_vector), self.sample_properties[1])
        filter_TF = np.abs(frequencies) <= cutoff # Lowpass Transfer Function
        self.Filtered_FFT = FFT_vector * filter_TF

        return self.Filtered_FFT

    # User will call this function to produce a spectrum that gets fed into the RNN
    def Filter_Signal(self):
        self.FFT_Heart_Signal(self.heart_samples)
        self.Lowpass_Filter(self.FFT)

        return self.Filtered_FFT

    # Functions for debugging and analysis:

    # Plot FFT data (can be filtered or unfiltered)
    def plot_FFT(self, FFT_data):
        time_vector = np.linspace(0, len(FFT_data) * self.sample_properties[1], endpoint=False)
        FFT_vector = np.array(FFT_data)
        frequency_vector = fftfreq(len(FFT_data), self.sample_properties[1])[:len(FFT_data) // 2]
        plt.plot(frequency_vector, 2 / len(FFT_data) * np.abs(FFT_data[0 : len(FFT_vector) // 2]))
        plt.grid()
        plt.show()

    # Plot reconstructed heartbeat data
    def plot_heartbeat(self, data):
        time_vector = np.linspace(0, len(data) * self.sample_properties[1], endpoint=False)
        plt.plot(time_vector, np.array(data))
        plt.grid()
        plt.show()
