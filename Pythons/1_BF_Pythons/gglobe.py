import numpy as np
import matplotlib.pyplot as plt
import argparse


def calc_response(mic_num, spacing, freq):

    ANGLE_RESOLUTION = 100
    speedSound = 340.29  # speed of the sound (m/s)

    tick = 360/ANGLE_RESOLUTION
    angles = np.arange(0,360+tick, tick)
    outputs = np.zeros(ANGLE_RESOLUTION+1)

    # calcurate the response of the mic array
    for a in range(ANGLE_RESOLUTION+1):
        angle = 360.0 * a / ANGLE_RESOLUTION
        angleRad = np.pi * angle / 180.0
        micsum = 0

        for n in range(mic_num):
            position = (n - (mic_num - 1) / 2) * spacing
            delay = position * np.sin(angleRad) / speedSound
            mfv = np.exp(-1j * 2.0 * np.pi * freq * delay)
            micsum = micsum + mfv

        # calcurate the decibel output
        output = np.abs(micsum) / mic_num
        logOutput = 20 * np.log10(output)
        if logOutput < -50:
            logOutput = -50
        outputs[a] = logOutput + 50


    plt.figure(figsize=(8, 6))
    plt.polar(np.radians(angles), np.abs(outputs))
    plt.title("Microphone Array Sensitivity")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--mic_num", type=int, default=4, help="number of mics")
    parser.add_argument("--space", type=float, default=0.1, help="space of mics")
    parser.add_argument("--freq", type=int, default=2000, help="frequency of the source signal")

    args = parser.parse_args()
    print(f"number of mics: {args.mic_num}")
    print(f"space of mics : {args.space}")
    print(f"frequency     : {args.freq}")

    calc_response(args.mic_num, args.space, args.freq)
