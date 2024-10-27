import numpy as np
import wave

sampling_rate = 48000
bit_length = 16

def binary2float(frames):
    global bit_length
    if bit_length == 16:
        data = np.frombuffer(frames, dtype=np.int16)
    elif bit_length == 24:
        sampwidth = bit_length/8
        a8 = np.frombuffer(frames, dtype=np.uint8)
        tmp = np.empty([length, 4], dtype=np.uint8)
        tmp[:, :sampwidth] = a8.reshape(-1, sampwidth)
        tmp[:, sampwidth:] = (tmp[:, sampwidth-1:sampwidth] >> 7) * 255
        data = tmp.view("int32")[:, 0]
    data = data.astype(float)/(2**(bit_length-1)) # Normalize (int to float)
    return data
 
def float2binary(data):
    data = (data*(2**(bit_length-1)-1)).reshape(data.size, 1) # Normalize (float to int)
    if bit_length == 16:
        frames = data.astype(np.int16).tobytes()
    elif bit_length == 24:
        a32 = np.asarray(data, dtype = np.int32)
        a8 = (a32.reshape(a32.shape + (1,)) >> np.array([0, 8, 16])) & 255
        frames = a8.astype(np.uint8).tobytes()
    return frames
 
def read_wave(file_name):
    file = wave.open(file_name, "rb") # open file
    sampwidth = file.getsampwidth()
    nframes = file.getnframes()
    frames = file.readframes(nframes)
    file.close() # close file
    return binary2float(frames) # binary to float
 
def write_wave(file_name, data):
    global sampling_rate
    global bit_length
    data = data / np.max(np.abs(data))
    detect = 0
    # print out the impulse response 
    # the impulse data starts 0.0324 sec. 0.0324x48000 = 1557 sample
    for index, num in enumerate(data):
        if index >= 1557 and index < 1557+128:
            print(f"{num:.8f},")
    file = wave.open(file_name, "wb") # open file
    # setting parameters
    file.setnchannels(1)
    file.setframerate(sampling_rate)
    file.setsampwidth(int(bit_length/8))
    frames = float2binary(data) # float to binary
    file.writeframes(frames)
    file.close() # close file
 
def gen_itsp(m = 18):
    N = 2**m + 2**(m-1)
    J = 2**m
    TSP = np.exp(-1j * (2*np.pi) * J * (np.arange(int(N/2)) / N)**2)  # generate TSP signal to make iTSP signal
    shift = int((N-J)/2)
    TSP = TSP * np.exp(-1j * (2*np.pi) * (shift/N) * np.arange(int(N/2))) # circular shift
    iTSP = 1 / TSP  # generate iTSP signal
    itsp = np.fft.irfft(iTSP)
    return itsp / np.max(np.abs(itsp))

def tspres2ir(tsp_res, itsp):
    N = itsp.shape[0]
    residual = tsp_res.shape[0] - N
    if residual >= N:
        tsp_res[:N]  = tsp_res[:N] + tsp_res[N:2*N]
    else:
        tsp_res[:residual] = tsp_res[:residual] + tsp_res[N:N+residual]
    TSP_RES = np.fft.rfft(tsp_res[:N])
    iTSP = np.fft.rfft(itsp)
    IR = TSP_RES * iTSP
    ir = np.fft.irfft(IR)
    return ir

if __name__=="__main__":
    tsp_res_path = "Sound.wav"
    ir_path = "ir.wav"
    m = 18
    tsp_res = read_wave(tsp_res_path)  # read the tsp response
    itsp = gen_itsp(m)                 # generate itsp signal
    ir = tspres2ir(tsp_res, itsp)      # make the impulse response
    ir = ir / np.max(ir)
    write_wave(ir_path, ir)
