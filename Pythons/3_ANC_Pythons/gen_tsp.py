import numpy as np
import wave

sampling_rate = 48000
bit_length = 16

def float2binary(data):
    global bit_length
    data = (data*(2**(bit_length-1)-1)).reshape(data.size, 1) # Normalize (float to int)
    if bit_length == 16:  # 16bit signal
        frames = data.astype(np.int16).tobytes()
    elif bit_length == 24:  # 24bit signal
        a32 = np.asarray(data, dtype = np.int32)
        a8 = (a32.reshape(a32.shape + (1,)) >> np.array([0, 8, 16])) & 255
        frames = a8.astype(np.uint8).tobytes()
    return frames
 
def write_wave(file_name, data):
    global sampling_rate
    global bit_length
    file = wave.open(file_name, "wb") # open file
    # setting parameters
    file.setnchannels(1)
    file.setsampwidth(int(bit_length/8))
    file.setframerate(sampling_rate)
    frames = float2binary(data) # float to binary
    file.writeframes(frames)
    file.close() # close file
 
def gen_tsp(m = 18):
    N = 2**m + 2**(m-1) # 2^18+2^17=393216 samples = 8.192sec 
    J = 2**m
    TSP = np.exp(-1j * (2*np.pi) * J * (np.arange(int(N/2)) / N)**2)
    shift = int((N-J)/2)
    TSP = TSP * np.exp(-1j * (2*np.pi) * (shift/N) * np.arange(int(N/2)))  # circular shift
    tsp = np.fft.irfft(TSP)
    return tsp / np.max(np.abs(tsp))

if __name__ == "__main__":
    m = 18                # parameter to define the pulse width
    tsp = gen_tsp(m)  # generate tsp signal
    write_wave("tsp_{}.wav".format(m), tsp)
