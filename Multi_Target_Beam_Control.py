# DPS libs
import numpy as np
import cmath
import math

# Arduino libs
import serial
import time

# Define a path for phase_set, imported file must be of type '.npy'
phase_set = np.load('C:\\Users\\Sythr\\OneDrive\\Documents\\BeamSteeringAlgorithm\\phase_set.npy')

# Convert the data into a 76 by 9 matrix with complex numbers
phase_set = np.array(phase_set, dtype = complex)

# Phased array parameters
nt = 4
dt = 0.5
K = 4

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

    w = np.dot(P_A, st[:, target_angle + 90]) # this and P_A are right, checked w/ matlab
    num = 76
    S = np.zeros([nt, num * num], dtype = complex)
    S_ang = np.zeros([nt, num * num])
    ang_ind = np.zeros([nt, num * num], dtype = int)

    # Choose largest-valued index for location of the maximum value
    m = max_index = max((i for i, x in enumerate(np.abs(w)) if x == max(np.abs(w))), default = None)

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
          w_appro[zz] = S[zz, ind[z, zz]]
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

# Send PWM and MUX commands to the arduino over serial
def send_data_to_arduino(PWM, MUX):

    # Send MUX commands to Arduino
    for i in range(0, 7):
        ser.write(f"{i} {MUX[i]}\n".encode())

    # Send PWM commands to Arduino
    for j in range(8, 15):
        ser.write(f"{j} {PWM[j - 8]}\n".encode())

# Main function for beam steering control
def main():

    # Define the serial port and baud rate
    serial_port = 'COM3'  # Update this with the appropriate port for your Arduino
    baud_rate = 9600

    # Open the serial connection
    arduino = serial.Serial(serial_port, baud_rate, timeout = 1)
    time.sleep(2)

    # Targets
    target_angles = [10, 40]

    # Monitoring time per target (seconds)
    monitoring_time = 10

    # Determine the phase shifter voltages
    for k in range(0, len(target_angles)):
        null_angles = list(target_angles)
        null_angles.remove(target_angles[k])
        voltages = DPS(target_angles[k], null_angles)
        print("DPS Voltages:", voltages)

        # Determine analog levels and MUX control commands
        quantized_voltages = quantize(voltages)
        analog_levels = quantized_voltages[0:7]
        MUX_control = quantized_voltages[8:15]
        print("Analog levels:", analog_levels)
        print("MUX control:", MUX_control)

        # Send data to arduino for control
        send_data_to_arduino(analog_levels, MUX_control)
        time.sleep(monitoring_time)

if __name__ == "__main__":
    main()
