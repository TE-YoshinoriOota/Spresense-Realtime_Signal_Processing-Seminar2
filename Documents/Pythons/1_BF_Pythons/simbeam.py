import numpy as np
import matplotlib.pyplot as plt
import argparse


def calc_response(mic_num, spacing, freq, target_angle):

    ANGLE_RESOLUTION = 100
    speedSound = 340.29  # speed of the sound (m/s)

    # initialize stearing vector 
    strVector = np.zeros((mic_num), dtype = np.complex64)
    tick = 360/ANGLE_RESOLUTION
    angles = np.arange(0,360+tick, tick)
    outputs = np.zeros(ANGLE_RESOLUTION+1)

    # deg -> rad
    target_angle_rad = np.pi * target_angle / 180.0

    # calcurate the mic position and the delay time
    for n in range(mic_num):
        position = (n - (mic_num - 1) / 2) * spacing
        delay = position * np.sin(target_angle_rad) / speedSound
        strVector[n] = np.exp(1j * 2.0 * np.pi * freq * delay) 

    # calcurate the response of the stearing vector to each degree
    for a in range(ANGLE_RESOLUTION+1):
        angle = 360.0 * a / ANGLE_RESOLUTION
        angleRad = np.pi * angle / 180.0
        micsum = 0

        for n in range(mic_num):
            position = (n - (mic_num - 1) / 2) * spacing
            delay = position * np.sin(angleRad) / speedSound
            mfv = np.exp(-1j * 2.0 * np.pi * freq * delay) # manifold vector
            micsum = micsum + strVector[n] * mfv

        # calcurate the log output
        output = np.abs(micsum) / mic_num
        logOutput = 20 * np.log10(output)
        if logOutput < -50:
            logOutput = -50
        outputs[a] = logOutput + 50


        # print(f"{a} {angle} {angleRad} {output} {logOutput}")

    plt.figure(figsize=(8, 6))
    plt.polar(np.radians(angles), np.abs(outputs))
    plt.title(f"Microphone Array Sensitivity (Beamforming at {target_angle} degree)")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--mic_num", type=int, default=4, help="number of mics")
    parser.add_argument("--space", type=float, default=0.1, help="space of mics")
    parser.add_argument("--freq", type=int, default=2000, help="frequency of the source signal")
    parser.add_argument("--angle", type=float, default=0.0, help="target angle of beamforming")

    args = parser.parse_args()
    print(f"number of mics: {args.mic_num}")
    print(f"space of mics : {args.space}")
    print(f"frequency     : {args.freq}")
    print(f"target angle  : {args.angle}")

    calc_response(args.mic_num, args.space, args.freq, args.angle)

