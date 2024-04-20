# DPS libs
import numpy as np
import cmath
import math
import matplotlib.pyplot as plt

# Arduino libs
#import serial
import time

# Define a path for phase_set, imported file must be of type '.npy'
# If this script and phase_set are in the same folder the path below should work
    # In vs code just make sure that you have the folder open in explorer
phase_set = np.load('phase_set.npy')

# Convert the data into a 76 by 9 matrix with complex numbers
phase_set = np.array(phase_set, dtype = complex)

# Phased array parameters
nt = 4 # Number of Transmitters
dt = 0.5 # Transmitter spacing / wavelength
K = 4
theta = np.arange(-90, 91, 1)
# Steering Matrix
temp = np.array(range(nt)).reshape(-1,1)*np.sin(theta*np.pi/180)
st = np.exp(-1j*2*np.pi*dt*temp)

# DPS function to determine steering voltages
def DPS(target_angle, null_angle):

    theta = np.arange(-90, 91, 1)
    st = np.exp(-1j * 2 * np.pi * dt * np.arange(nt)[:, np.newaxis] * np.sin(theta*np.pi/180))

    shifted_null_angles = null_angle
    for vals in null_angle:
        shifted_null_angles = vals + 90

    A = st[:, shifted_null_angles]

    # Check the shape of A
    if A.ndim == 1:
        A = A[:, np.newaxis]

    P_A = np.eye(nt) - np.dot(A, A.T) / nt
    conj_row = P_A[:, 0]
    conj_row = conj_row.conj()

    P_A = np.eye(nt, dtype = complex)

    P_A[0][0] = conj_row[0]
    P_A[0][1] = conj_row[1]
    P_A[0][2] = conj_row[2]
    P_A[0][3] = conj_row[3]

    P_A[1][0] = conj_row[1].conj()
    P_A[1][1] = conj_row[0]
    P_A[1][2] = conj_row[1]
    P_A[1][3] = conj_row[2]

    P_A[2][0] = conj_row[2].conj()
    P_A[2][1] = conj_row[1].conj()
    P_A[2][2] = conj_row[0]
    P_A[2][3] = conj_row[1]

    P_A[3][0] = conj_row[3].conj()
    P_A[3][1] = conj_row[2].conj()
    P_A[3][2] = conj_row[1].conj()
    P_A[3][3] = conj_row[0]

    w = np.dot(P_A, st[:, target_angle + 90])
    num = 76
    S = np.zeros([nt, num * num], dtype = complex)
    S_ang = np.zeros([nt, num * num])
    ang_ind = np.zeros([nt, num * num], dtype = int)

    # Choose largest-valued index for location of the maximum value
    m = max((i for i, x in enumerate(np.abs(w)) if x == max(np.abs(w))), default = None)

    temp = np.zeros([76, 76], dtype = complex)
    for i in range(0, 4):
        temp_col = phase_set[:, 2 * i]
        temp_col_neigh = phase_set[:, 2*i + 1]
        for j in range(0, 76):
            for k in range(0, 76):
                temp[j][k] = temp_col[j] + temp_col_neigh[k]

        S[i][:] = temp.T.reshape(1, -1)
        S_ang[i][:] = np.angle(S[i][:])
        
        ang_ind[i][:] = np.argsort(S_ang[i][:])
        S_ang[i][:] = np.sort(S_ang[i][:])

    resi = num**2 // K
    ind = np.zeros((K, nt), dtype=int)
    for k in range(0,K):
        j = range(k * resi, (k+1)*resi)
        g = ang_ind[m, j]
        Smk = S[m, g]
        
        max_ind = np.argmax(np.abs(Smk))
        alpha = Smk[max_ind] / w[m]
     
        for n in range(nt):
            if m == n:
                ind[k, n] = ang_ind[m, max_ind + k*resi]
            temp = w[n]*alpha
            for j in range(K):
                Snk_min = S[n, ang_ind[n, j*resi]]
                Snk_max = S[n, ang_ind[n, (j+1)*resi-1]]
                if np.angle(temp) >= np.angle(Snk_min) and np.angle(temp) <= np.angle(Snk_max):
                    break
            n_ind = np.argmin(np.abs(S[n, ang_ind[n, range(j*resi,(j+1)*resi)]] - temp))
            ind[k, n] = ang_ind[n, n_ind + j*resi]

    # Index correction
    ind = ind + 1

    # Choose the best solution w.r.t. null depth
    null_depth = np.zeros([K, 1])

    # Convert angles to radians
    target_angle_rad = np.radians(target_angle)
    null_angle_rad = np.radians(null_angle)

    # Compute st matrix
    st = np.exp(-1j * 2 * np.pi * 0.5 * np.outer(np.arange(nt), np.sin(target_angle_rad * np.pi / 180)))

    for z in range(K):
        w_appro = np.zeros((nt, 1), dtype = complex)
        for zz in range(nt):
            w_appro[zz] = S[zz, ind[z, zz] - 1]
        bp = np.abs(np.dot(w_appro.T.conj(), st))**2
        bp = 10 * np.log10(bp)
        bp = bp - np.max(bp)
        null_depth[z] = 2 * np.mean(bp[0] - bp[0,:])

    final_k = np.argmax(null_depth)

    ind2 = np.ceil(ind[final_k] / 76)
    ind1 = ind[final_k] - (ind2 - 1) * 76

    # Voltage computation
    v1 = (ind1 - 1) * 0.2
    v2 = (ind2 - 1) * 0.2

    # Compress voltages into an array
    voltage = [round(val, 1) for pair in zip(v1, v2) for val in pair]

    return voltage

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

# Plots the Beampattern of the voltages
def Beampattern(voltages, target_angles, flag = False):
    # Inputs
        # voltages: Matrix of target voltages for phase shifter
        # target_angles: Angles that the beam will be steering between
        # flag: If False the plots will be separated if True the plots will be together
            # If no input it will assume the input is False


    # Converts voltages to numpy array to be able to index
    v = np.array(voltages)
    # Goes through the different sets of angles
    for row_index in range(len(target_angles)):
        # Calculating ind1 and ind2 (originally calculated in the DPS function)
        ind1 = [round(v[row_index, x]/0.2 + 1) for x in [0,2,4,6]]
        ind2 = [round(v[row_index, x]/0.2 + 1) for x in [1,3,5,7]]
        
        w_appro = []
        for i in range(nt):
            w_appro.append(phase_set[ind1[i]-1, 2*i] + phase_set[ind2[i]-1, 2*i+1])
        w_appro = np.array(w_appro)

        # Finds the beam patters, converts to decibels and normalizes it
        bp = np.abs(np.dot(w_appro.T, st))**2
        bp_p = 10 * np.log10(bp)
        bp_dps = bp_p - np.max(bp_p)
        
        # Separate beampattern plots
        if flag == False:
            plt.figure()
            
            # Plots the beam pattern
            plt.plot(-theta, bp_dps, '--', linewidth=2, label= 'Beam Pattern')
            plt.xlabel('Degrees')
            plt.ylabel('dB')
            plt.title(f'Beam Pattern Target Angle: {target_angles[row_index]}')

            # Adds vertical lines plotting the target angle in green and the null angles in red
            plt.axvline(x = target_angles[row_index], color = 'g', label = 'Target Angle')
            null_angles = list(target_angles)
            null_angles.remove(null_angles[row_index])
            
            if len(null_angles) == 1:
                plt.axvline(x = angle, color = 'r', label = 'Null Angle')
            else:
                null_angles_added = False
                for angle in null_angles:
                    plt.axvline(x = angle, color = 'r', label='Null Angle' if not null_angles_added else '')
                    null_angles_added = True
            # Adds the legend
            plt.legend(loc='lower left')

        # All beampatterns in one
        elif flag == True:
            # Plots the beam pattern
            plt.plot(-theta, bp_dps, '--', linewidth=2, label=f'Target Angle: {target_angles[row_index]}')
            plt.xlabel('Degrees')
            plt.ylabel('dB')
            plt.title('Beam Pattern')

            # Adds vertical lines plotting the target angle in green and the null angles in red
            for angle in target_angles:
                plt.axvline(x = angle, color = 'k')
            # Adds the legend
            plt.legend(loc='lower left')


        else:
            print("Invalid call to beampattern. The third input should be:")
            print("\t False: Indicating you want all beampatterns to be in individual plots.")
            print("\t True: Indicating you want all beampatterns in one")
            print("\t If no third input it will assume False")

    # Displays the plots
    plt.show()

        
# Main function for beam steering control
def main():
    '''
    # Define the serial port and baud rate
    serial_port = 'COM3'  # Update this with the appropriate port for your Arduino
    baud_rate = 9600

    # Open the serial connection
    arduino = serial.Serial(serial_port, baud_rate, timeout = 1)

    # Wait for the Arduino to reset
    time.sleep(2)
    '''

    # Targets
    target_angles = [-89, -50,-10, 30, 65, 90]

    # Store DPS voltages for each target
    voltage_storage = []
    
    # Call the function and print the result
    for val in range(0, len(target_angles)):
        null_angle = list(target_angles)
        null_angle.remove(null_angle[val])
        temp_voltages = DPS(target_angles[val], null_angle)
        print(temp_voltages)
        voltage_storage.append(list(temp_voltages))

    # Plot the BeamPattern (Comment out if you don't need to use it)
        # If no third input or false it prints each beam pattern separately
        # If third input is True it plots all beam patterns together
    Beampattern(voltage_storage, target_angles, True)

    # Monitoring time per target (seconds)
    monitoring_time = 10

    # Total number of monitoring cycles
    monitoring_cycles = 1

    '''
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
            '''

if __name__ == "__main__":
    main()