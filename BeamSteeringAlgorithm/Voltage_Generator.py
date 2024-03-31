# DPS libs
import numpy as np
import math
from oct2py import octave

# Arduino libs
import serial
import time

# DPS function to determine steering voltages
def call_octave_dps(target_angle, null_angle):
    # Call the DPS function in Octave
    try:
        voltages = octave.DPS(target_angle, null_angle)
    except Exception as e:
        print("Error calling DPS function in Octave:", e)
        voltages = None
    return voltages

# Converts phase shifter voltages to their corresponding quantization level
def quantize(voltages):

    analog_levels = [0, 0, 0, 0, 0, 0, 0, 0]
    MUX_control = [0, 0, 0, 0, 0, 0, 0, 0]

    for i in range(0, len(voltages)):
        if voltages[i] >= 1:
            analog_levels[i] = math.floor((voltages[i] - 15) / (-0.0547))
            MUX_control[i] = 0
        else:
            analog_levels[i] = math.floor(voltages[i] / 51)
            MUX_control[i] = 1

    quantized_data = analog_levels + MUX_control
    return quantized_data

# Main function for beam steering control
def main():

    # Define the serial port and baud rate
    serial_port = 'COM3'  # Update this with the appropriate port for your Arduino
    baud_rate = 9600

    # Open the serial connection
    arduino = serial.Serial(serial_port, baud_rate, timeout = 1)

    # Wait for the Arduino to reset
    time.sleep(2)

    # Targets
    target_angles = [40, 20]

    # Store DPS voltages for each target
    voltage_storage = []
    
    # Load the "DPS.m" file into the Octave workspace
    octave.addpath('C:\\Users\\Sythr\\OneDrive\\Documents\\BeamSteeringAlgorithm')  # Add the directory containing DPS.m if necessary
    octave.eval('warning("off", "Octave:data-file-in-path")')  # Suppress the specific warning
    
    # Modify the path to the correct location of the "DPS.m" file
    try:
        octave.eval('source("C:\\Users\\Sythr\\OneDrive\\Documents\\BeamSteeringAlgorithm\\DPS.m")')
    except Exception as e:
        pass
    
    # Call the function and print the result
    for val in range(0, len(target_angles)):
        null_angle = list(target_angles)
        null_angle.remove(null_angle[val])
        temp_voltages = call_octave_dps(target_angles[val], null_angle)
        voltage_storage.append(temp_voltages[0])

    # Monitoring time per target (seconds)
    monitoring_time = 10

    # Total number of monitoring cycles
    monitoring_cycles = 3

    # Determine the phase shifter voltages
    for cycles in range(monitoring_cycles):
        for k in range(0, len(target_angles)):
            voltages = voltage_storage[k]
            print("DPS Voltages:", voltages)

            # Determine analog levels and MUX control commands
            quantized_voltages = quantize(voltages)
            analog_levels = quantized_voltages[0:7]
            MUX_control = quantized_voltages[8:15]
            print("Analog levels:", analog_levels)
            print("MUX control:", MUX_control)

            # Send data to arduino for control
            
            # Send MUX commands to Arduino
            for i in range(0, 7):
                arduino.write(f"{i} {MUX_control[i]}\n".encode())

            # Send PWM commands to Arduino
            for j in range(8, 15):
                arduino.write(f"{j} {analog_levels[j - 8]}\n".encode())

            time.sleep(monitoring_time)

if __name__ == "__main__":
    main()
