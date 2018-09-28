import numpy as np
import time
import threading
from math import cos,sin,radians

#imports for class stepper
import sys
import time

KR = 0.2 # Constante de rozamiento
KR_EDGE = 1 # Constante de rozamiento cerca del borde #TODO
EDGES = np.array([[-1000,1000],[-1000,1000]]) # Bordes xmin,xmax,ymin,ymax
MASS = 1 #mass
DELAY = 0.01

PINS_PAN= [17,22,23,24]
PINS_TILT= [99,99,99,99] #TODO corregir

# class created to run this module without GPIO library
class GPIO_DUMMY(object):
    def __init__(self, *args, **kwargs):
        self.BCM = 0
        self.OUT = 0
    def setmode(self,*args):
        pass
    def setup(self,*args):
        pass
    def output(self,*args):
        pass

try:
    import RPi.GPIO as GPIO
    gpio_enabled = True
except:
    GPIO = GPIO_DUMMY()
    gpio_enabled =  False
    print("Unable to load RPi.GPIO")

class Stepper(object):
    def __init__(self,pins,delay=DELAY*1000):
        self._StepPins = pins
        self._x = 0
        # Use BCM GPIO references
        # instead of physical pin numbers
        GPIO.setmode(GPIO.BCM)
        # Set all pins as output
        for pin in self._StepPins:
            print("Setup pins")
            GPIO.setup(pin,GPIO.OUT)
            GPIO.output(pin, False)
        # Define advanced sequence
        # as shown in manufacturers datasheet
        self._Seq = [[1,0,0,1],
            [1,0,0,0],
            [1,1,0,0],
            [0,1,0,0],
            [0,1,1,0],
            [0,0,1,0],
            [0,0,1,1],
            [0,0,0,1]]
            
        self._StepCount = len(self._Seq)
        # Read wait time
        if delay>10:
            self._WaitTime = int(delay)/float(1000)
        else:
            self._WaitTime = 10/float(1000)
        # Initialise variables
        self._StepCounter = 0
    
    def __del__(self):
        # go back to the original position
        self.steps(-self._x)

    def step(self,dir=1,delay=None):
        assert(dir==1 or dir==-1)
        if delay !=0:
            if delay is None:
                delay = self._WaitTime
            time.sleep(delay)
        self._StepCounter += dir
        self._x +=dir
        # If we reach the end of the sequence
        # start again
        if (self._StepCounter>=self._StepCount):
            self._StepCounter = 0
        if (self._StepCounter<0):
            self._StepCounter = self._StepCount+dir
        print("Current position x=",self._x)
        print(self._Seq[self._StepCounter])
        for pin in range(0, 4):
            xpin = self._StepPins[pin]
            if self._Seq[self._StepCounter][pin]!=0:
                print(" Enable GPIO %i" %(xpin))
                GPIO.output(xpin, True)
            else:
                GPIO.output(xpin, False)

    def steps(self,steps,delay=None):
        if steps < 0:
            dir = -1
        else:
            dir = 1
        for i in range(abs(steps)):
            self.step(dir,delay)


class PanTilt():
    def __init__(self, steppers=None,*args, **kwargs):
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
        self.set_steppers(steppers)

    def __del__(self):
        """Return to initial position"""
        if self._steppers is not None:
            pass
            #TODO

    def _read_position(self,channel):
        pass
        # TODO
            
    def set_steppers(self,steppers):
        if steppers is None:
            self._steppers = None
        else:
            if isinstance(steppers[0],Stepper) and isinstance(steppers[1],Stepper):
                self._steppers = steppers
            else:
                raise TypeError("stepper should be of type Stepper")

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
            #TODO: considerar la posibilidad de dejar fijo el dt
            self._dt = self._t - prev_t
            # self._dt=0.01
            self._update()
            #print(f"Location: {self._x}, velocity: {self._v}, acceleration: {self._a}, dt: {dt}")
            print("Location: x{}, velocity: {}, acceleration: {}, dt: {}".format(self._x,self._v,self._a,self._dt))
            if self._steppers is not None:
                for i in range(0,2):
                    xi = int(round(self._x[i]))
                    if xi != self._steppers[i]._x:
                        self._steppers[i].steps(xi - self._steppers[i]._x,0)
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
    # import Adafruit_PCA9685
    # pwm = Adafruit_PCA9685.PCA9685()
    # pantilt = PanTilt(pwm)

    # pantilt = PanTilt()
    # pantilt.move(50,45)
    # time.sleep(1)
    # pantilt.move(10,270)
    # time.sleep(1)
    # pantilt.stop()

    pan = Stepper(PINS_PAN)
    tilt = Stepper(PINS_TILT)
    steppers = (pan,tilt)
    pantilt = PanTilt(steppers)
    pantilt.move(10,30)


if __name__ == "__main__":
    main()      

