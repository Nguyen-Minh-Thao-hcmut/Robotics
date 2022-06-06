import math
from scipy import signal

class butterworth_filter:
    def __init__(self, f_sample = 40000, f_pass = 4000, f_stop = 8000, fs = 0.5, Td = 1, g_pass = 0.5, g_stop = 40, type = 'low', analog = False):
        self.f_sample = f_sample
        self.wp = f_pass/(f_sample/2)
        self.ws = f_stop/(f_sample/2)
        self.omega_p = (2/Td)*math.tan(self.wp/2)
        self.omega_s = (2/Td)*math.tan(self.ws/2)
        self.N, self.Wn = signal.buttord(self.wp, self.ws, g_pass, g_stop,analog = analog)
        self.b, self.a = signal.butter(self.N, self.Wn, type,analog)
        self.z, self.p = signal.bilinear(self.b, self.a, fs)

    def apply_filter(self,input):

        output = signal.filtfilt(self.b, self.a, input)
        return output

        