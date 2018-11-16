import numpy as np
import time
import threading
from math import cos,sin,radians
import code #for debug
#usage: pan = Servo(0)
# pan.goto(x1) move servo to position x1

#NOTE: servo response time is approx 0.002 secs

A_MAX = 100*5
V_MAX = 50*5*2
X_MAX = 1990
X_MIN = 1650
DELAY = 0.02
FREQUENCY = 300

class PanTilt:
    def __init__(self,nopwm=False):
        # exec(open("smooth_servo.py").read())
        if nopwm is False:
            import Adafruit_PCA9685
            pwm = Adafruit_PCA9685.PCA9685()
            pwm.set_pwm(0,0,round(6*FREQUENCY))
            pwm.set_pwm(1,0,round(5*FREQUENCY))
        else:
            pwm=None
        self.pan = Servo(pwm,0)
        self.tilt = Servo(pwm,1)
        self.pan.set_parameters(max_x=2450,min_x=1650)
        self.pan.moveto(2000)
        self.tilt.set_parameters(max_x=1900,min_x=750)
        self.tilt.moveto(1200)
    
    def move(self,angle,force):
        if force>10:
            force=10
        forcex = cos(radians(angle))*force
        forcey = sin(radians(angle))*force
        xf = (1, -1)[forcex < 0]*10000
        yf = (1, -1)[forcey < 0]*10000
        ax = abs(forcex)*A_MAX/2/10+A_MAX/2
        ay = abs(forcey)*A_MAX/2/10+A_MAX/2
        vx = abs(forcex)*V_MAX/10
        vy = abs(forcey)*V_MAX/10
        self.pan.goto(xf,ax,vx)
        self.tilt.goto(yf,ay,vy)

class Servo:
    def __init__(self, pwm=None, defaultchannel=0):
        self._channel = defaultchannel # 0 for pan 1 for tilt
        self._pwm = None
        self._a = 0 # acceleration
        self._v = 0 # velocity
        self._x = 0 # current position
        self._xf = 0 # position we are going
        self._epsilon = 0.001 #small constant
        self._t = time.time() #current time
        self._a_max = A_MAX
        self._v_max = V_MAX
        self._x_max = X_MAX
        self._x_min = X_MIN
        self._delay = DELAY
        self._frequency = FREQUENCY
        self._moving = False
        self.set_parameters()
        self.set_pwm(pwm)
    
    def __del__(self):
        """Sends a software reset (SWRST) command to all servo drivers on the bus. and put to sleep"""
        if self._pwm is not None:
            #self._pwm._device.writeRaw8(0x06)  # SWRST NO SE SI ES NECESARIO, quizas mejor quitar
            self._pwm._device.write8(0x00, 0x10)  #put to sleep
    
    def set_position(self,x=0,v=0,a=0):
        self._x = x
        self._v = v
        self._a = a

    def read_position(self,channel):
        if self._pwm is not None:
            byte_L = self._pwm._device.readU8(0x08+4*channel)
            byte_H = self._pwm._device.readU8(0x09+4*channel)
            self._x = byte_H*256+byte_L
            return self._x
        
    def set_pwm(self,pwm):
        if pwm is None:
            self._pwm = None
        else:
            try:
                from Adafruit_PCA9685 import PCA9685
                if isinstance(pwm,PCA9685):
                    self._pwm = pwm
                    self.read_position(self._channel) #DA ERROR-> CORREGIR
                    self._pwm.set_pwm_freq(self._frequency)
                else:
                    raise TypeError("PWM should be of type PCA9685")
            except ImportError:
                raise ImportError("This library requires Adafruit_PCA9685\nInstall with: sudo pip3 install adafruit-pca9685")
    
    def set_channel(self,channel):
        ''' Set default channel '''
        self._channel = channel
    
    def set_input(self,xf,max_velocity=None,max_acceleration=None):
        if max_velocity is not None:
            self._v_max = max_velocity
            if max_velocity>V_MAX:
                self._v_max = V_MAX
        if max_acceleration is not None:
            self._a_max = max_acceleration
            if max_acceleration>A_MAX:
                self._a_max = A_MAX
        if xf>self._x_max:
            self._xf = self._x_max
        elif xf<self._x_min:
            self._xf = self._x_min
        else:
            self._xf = xf

    def set_parameters(self,max_acceleration=A_MAX, max_velocity=V_MAX, delay=DELAY, max_x=X_MAX, min_x=X_MIN,frequency=None):
        self._a_max = max_acceleration
        self._v_max = max_velocity
        if max_velocity>V_MAX:
            self._v_max = V_MAX
        self._x_max = max_x
        self._x_min = min_x
        self._delay = delay
        self._dx_max = self._v_max**2/2/self._a_max
        self._dx_epsilon = self._epsilon * max_x * 1.0
        self._dv_epsilon = self._epsilon * max_x * 10.0
        if self._pwm is not None and frequency is not None:
            self._frequency = frequency
            self._pwm.set_pwm_freq(self._frequency)

    def _stop_check_x(self):
        if (abs(self._xf-self._x)<self._dx_epsilon and abs(self._v)<self._dv_epsilon) or (self._v_max<self._epsilon or self._a_max<self._epsilon):
            return True
        else:
            return False
    
    def _update(self,dt):

        # Acceleration calculation
        dx = self._xf - self._x
        sign_dx = (1, -1)[dx < 0]
        sign_v = (1,-1)[self._v<0]
        # If velocity is in the other direction, set accelaration towards goal.
        if sign_v != sign_dx:
            self._a= self._a_max * sign_dx
        else:
            # Case velocity is towards goal
            # rozamiento proporcional al cuadrado de la velocidad
            dxfreno = self._v**2/2/self._a_max + self._dx_epsilon/2
            if abs(dx)>max(self._dx_max,dxfreno):
                # Case distance higher than dx_max
                if abs(self._v)>self._v_max:
                    # Case faster than v_max
                    self._a = 0
                else:
                    self._a = self._a_max * sign_dx
            else:
                if abs(dx) > dxfreno:
                    # accelerate towards goal
                    self._a = self._a_max * sign_dx
                else:
                    # deccelerate
                    self._a = -1*self._a_max * sign_dx
        
        # Update velocity
        self._v = self._v + self._a * dt

        # Update x
        self._x = self._x + self._v * dt + self._a/2*dt*dt
        
    def _goto(self,channel=None):
        if channel is None:
            channel=self._channel
        self._t = time.time()
        self._moving= True
        while not self._stop_check_x():
            time.sleep(self._delay)
            prev_t = self._t
            self._t = time.time()
            dt = self._t - prev_t
            self._update(dt)
            #print(f"Location: {self._x}, velocity: {self._v}, acceleration: {self._a}, dt: {dt}")
            print("Location: x{} xf{}, velocity: {}, acceleration: {}, dt: {}".format(self._x,self._xf,self._v,self._a,dt))
            if self._pwm is not None:
                self._pwm.set_pwm(channel, 0, round(self._x))
                print("pwm: ",self._pwm._device._address)
        self._moving = False
        return True
    
    # def goto(self,x1,v_max=None,a_max=None,channel=None):
    #     self.set_input(x1,v_max,a_max)
    #     if self._moving is False:
    #         task = threading.Thread(target=self._goto, args=(channel,))
    #         task.start()
    
    # def moveto(self,x1,channel=None):
    #     if channel is None:
    #         channel=self._channel
    #     if self._pwm is not None:
    #         self._pwm.set_pwm(channel, 0, round(x1))
    #         self._x = x1
    #         time.sleep(2)
            


def main():
    # exec(open("smooth_servo.py").read())
    # vmax = 430 # full left 60hz
    # vmin = 280
    # import Adafruit_PCA9685
    # pwm = Adafruit_PCA9685.PCA9685()
    
    pantilt = PanTilt()
    pantilt.move(90,1)
    # code.InteractiveInterpreter()
    # code.interact()

    # pan = Servo()
    # pan._x = 2500
    # pan.goto(2000)
    

    # frequency = 300
    # pwm.set_pwm_freq(frequency)
    # pwm.set_pwm(0,0,round(300*frequency/60))
    # time.sleep(1)
    
    # pantilt.pan.goto(2100)
    # time.sleep(0.5)
    # pantilt.pan.goto(1600)
    # code.interact(local=dict(globals(), **locals()))

if __name__ == "__main__":
    main()
