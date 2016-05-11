import wave
import numpy as np

f = wave.open('heart-threebeats.wav')
nframes = f.getnframes()
frames = f.readframes(nframes)

idxdata = np.zeros(len(frames)/2, np.float32)
for i in range(len(frames)/2):
    msb = np.uint8(ord(frames[2*i+1]))
    lsb = np.uint8(ord(frames[2*i]))
    s_comb = np.int16(msb<<8|lsb);
    idxdata[i] = np.float32(s_comb)
    
    # print idxdata[i]

f.close()
    
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api

fs, data = wavfile.read('heart-threebeats.wav') # load the data
a = data # this is a two channel soundtrack, I get the first track
b=[(ele/2**8.)*2-1 for ele in a] # this is 8-bit track, b is now normalized on [-1,1)
c = fft(b) # calculate fourier transform (complex numbers list)
d = len(c)/2  # you only need half of the fft list (real signal symmetry)
plt.plot(abs(c[:(d-1)]),'r') 
plt.show()

a = np.uint8(0b00000000)
b = np.uint8(0b00000001)
c = np.uint8(0b11000111)
d = np.uint8(0b11010110)
