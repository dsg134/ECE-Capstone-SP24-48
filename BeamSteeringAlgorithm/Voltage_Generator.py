import numpy as np

def DPS(target_angle, null_angle):
    # Load phase set
    phase_set = np.load('phase_set.npy')
    set = phase_set

    # Global Parameters
    nt = 4  # number of Transmitters
    dt = 0.5  # Transmitter spacing / wavelength
    K = 4  # how many sets of solutions to compare

    theta = np.arange(-90, 91, 1)
    st = np.exp(-1j * 2 * np.pi * dt * np.outer(np.arange(nt), np.sin(np.deg2rad(theta))))  # steering matrix

    # Beamformer
    A = st[:, null_angle + 90]
    P_A = np.eye(nt) - A @ np.linalg.inv(A.conj().T @ A) @ A.conj().T
    w = P_A @ st[:, target_angle + 90]

    m = np.argmax(np.abs(w))
    num = set.shape[0]
    S = np.zeros((nt, num ** 2), dtype=np.complex128)
    S_ang = np.zeros_like(S, dtype=np.float64)
    ang_ind = np.zeros_like(S, dtype=np.int64)
    for i in range(nt):
        temp = set[:, 2 * i - 1] + set[:, 2 * i].T
        S[i, :] = temp.flatten()
        S_ang[i, :] = np.angle(S[i, :])
        ang_ind[i, :] = np.argsort(S_ang[i, :])

    resi = num ** 2 // K
    ind = np.zeros((K, nt), dtype=np.int64)
    for k in range(K):
        Smk = S[m, ang_ind[m, (k * resi):((k + 1) * resi)]]
        max_ind = np.argmax(np.abs(Smk))
        alpha = Smk[max_ind] / w[m]
        for n in range(nt):
            if m == n:
                ind[k, n] = ang_ind[m, max_ind + k * resi]
                continue
            temp = w[n] * alpha
            for j in range(K):
                Snk_min = S[n, ang_ind[n, j * resi]]
                Snk_max = S[n, ang_ind[n, (j + 1) * resi - 1]]
                if np.angle(temp) >= np.angle(Snk_min) and np.angle(temp) <= np.angle(Snk_max):
                    break
            n_ind = np.argmin(np.abs(S[n, ang_ind[n, j * resi]:((j + 1) * resi)] - temp))
            ind[k, n] = ang_ind[n, n_ind + j * resi]

    null_depth = np.zeros((K,))
    st = np.exp(-1j * 2 * np.pi * 0.5 * np.array([0, 1])[:, np.newaxis] * np.sin(np.deg2rad([target_angle, null_angle])))

    for z in range(K):
        w_appro = np.array([S[zz, ind[z, zz]] for zz in range(nt)])
        bp = np.abs(w_appro.conj().T @ st)**2
        bp = 10 * np.log10(bp)
        bp = bp - np.max(bp)
        null_depth[z] = np.mean(bp[0] - bp[1:])

    final_k = np.argmax(null_depth)
    ind2 = np.ceil(ind[final_k, :] / 76).astype(int)
    ind1 = ind[final_k, :] - (ind2 - 1) * 76

    v1 = (ind1 - 1) * 0.2
    v2 = (ind2 - 1) * 0.2
    voltage = np.concatenate((v1, v2))

    return voltage

angles = [5, 25, 30]
Voltages = np.zeros((len(angles), 152))

for i in range(len(angles)):
    # DPS (magnitude first search)
    null_angles = angles[:]
    null_angles.pop(i)
    Voltages[i, :] = DPS(angles[i], null_angles)

print(Voltages)

# SteerBeamOnArduino(port, Voltages)
