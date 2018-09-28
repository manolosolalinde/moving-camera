import numpy as np
import time
import threading
from math import cos,sin,radians

KR = 0.2 # Constante de rozamiento
KR_EDGE = 1 # Constante de rozamiento cerca del borde #TODO
EDGES = np.array([[1650,2420],[750,1900]]) # Bordes xmin,xmax,ymin,ymax
MASS = 1 #mass
DELAY = 0.01

class PanTilt():
    def __init__(self, pwm=None,*args, **kwargs):
        self._x = np.array([0.0,0.0])
        self._a = np.array([0.0,0.0])
        self._v = np.array([0.0,0.0])
        self._f = np.array([0.0,0.0])
        self._t = time.time()
        self._dt = 0.001
        self._delay = DELAY
        self._m = MASS
        self._edges = EDGES
        self._x[0]=(self._edges[0,0]+self._edges[0,1])/2
        self._x[1]=(self._edges[1,0]+self._edges[1,1])/2
        self._margin = np.array([50,50])
        self._moving=False
        self._frequency = 300 # device freq in Hz
        self.set_pwm(pwm)

    def _read_position(self,channel):
        if self._pwm is not None:
            byte_L = self._pwm._device.readU8(0x08+4*channel)
            byte_H = self._pwm._device.readU8(0x09+4*channel)
            x = byte_H*256+byte_L
            return x

    def __del__(self):
        """Sends a software reset (SWRST) command to all servo drivers on the bus. and put to sleep"""
        if self._pwm is not None:
            #self._pwm._device.writeRaw8(0x06)  # SWRST NO SE SI ES NECESARIO, quizas mejor quitar
            self._pwm._device.write8(0x00, 0x10)  #put to sleep

    def set_pwm(self,pwm):
        if pwm is None:
            self._pwm = None
        else:
            try:
                from Adafruit_PCA9685 import PCA9685
                if isinstance(pwm,PCA9685):
                    self._pwm = pwm
                    self._pwm.set_pwm_freq(self._frequency)
                else:
                    raise TypeError("PWM should be of type PCA9685")
            except ImportError:
                raise ImportError("This library requires Adafruit_PCA9685\nInstall with: sudo pip3 install adafruit-pca9685")

    def _Hramp(self,v):
        v0=v.copy() # +0 or copy() is important to create a new array. otherwise v is passed as reference.
        v0[v0>1]=1
        v0[v0<0]=0
        return v0
    
    def _update(self):
        self._a = self._f/self._m - KR * self._Hramp(self._v)
        self._v += self._a * self._dt
        self._x += self._v * self._dt
        # edge correction
        if self._edges[0,0] > self._x[0]:
            self._x[0]=self._edges[0,0]
            self._v[0]=0
        elif self._edges[0,1] < self._x[0]:
            self._x[0]=self._edges[0,1]
            self._v[0]=0
        if self._edges[1,0] > self._x[1]:
            self._x[1]=self._edges[1,0]
            self._v[1]=0
        elif self._edges[1,1] < self._x[1]:
            self._x[1]=self._edges[1,1]
            self._v[1]=0
    
    def _stop_check(self):
        if self._moving is True:
            return False
        else:
            return True

    def _begin(self):
        self._t = time.time()
        self._moving=True
        while not self._stop_check():
            time.sleep(self._delay)
            prev_t = self._t
            self._t = time.time()
            self._dt = self._t - prev_t
            # self._dt=0.01
            self._update()
            #print(f"Location: {self._x}, velocity: {self._v}, acceleration: {self._a}, dt: {dt}")
            print("Location: x{}, velocity: {}, acceleration: {}, dt: {}".format(self._x,self._v,self._a,self._dt))
            if self._pwm is not None:
                self._pwm.set_pwm(0, 0, int(round(self._x[0])))
                self._pwm.set_pwm(1, 0, int(round(self._x[1])))
                #print("pwm: ",self._pwm._device._address)
        return True

    def stop(self):
        self._moving = False

    def move(self,force,angle):
        self._update_input(force,angle)
        if self._moving is False:
            task = threading.Thread(target=self._begin)
            task.start()
    
    def _update_input(self,force,angle):
        self._f[0] = cos(radians(angle))*force
        self._f[1] = sin(radians(angle))*force

def main():
    import Adafruit_PCA9685
    pwm = Adafruit_PCA9685.PCA9685()

    pantilt = PanTilt(pwm)
    pantilt.move(50,45)
    time.sleep(2)
    pantilt.stop()
    # pantilt._update_input(10,45)
    # pantilt._begin()

if __name__ == "__main__":
    main()      