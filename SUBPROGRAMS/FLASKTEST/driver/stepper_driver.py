import threading
import time
from math import cos,sin,radians
import numpy as np
import code

CHECK_DELAY = 0.001 #must be lower than MIN_DELAY
MIN_DELAY = 0.001 # Default delay in seconds
MAX_DELAY = 0.5 # Max delay in seconds
MAX_DURATION = 0.01 #seconds to wait before stop

PINS_PAN= [17,18,27,22]
PINS_TILT= [26,19,13,16]


#class created to run this module without GPIO library
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
    def __init__(self, pins):
        self._delay = MIN_DELAY
        self._duration= 4 #loops
        self._running = False
        self._StepPins = pins
        self._x = 0
        self._dir = 1
        self._task = None
        self._enable_commands = True #not used
        self._writing_lock = threading.Lock()
        # Use BCM GPIO references
        # instead of physical pin numbers
        # global GPIO #not useful
        self._GPIO = GPIO
        self._GPIO.setmode(GPIO.BCM)
        # Set all pins as output
        for pin in self._StepPins:
            # print("Setup pins")
            self._GPIO.setup(pin,GPIO.OUT)
            self._GPIO.output(pin, False)
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
        self._WaitTime = 10/float(1000)
        # Initialise variables
        self._StepCounter = 0
    
    def shutdown(self):
        if self._task is not None:
            if self._task.isAlive():
                self._task.join()
        self.goto(0,MIN_DELAY)
        self._GPIO.setmode(GPIO.BCM)
        # Set all pins as output
        for pin in self._StepPins:
            # print("Setup pins")
            self._GPIO.setup(pin,GPIO.OUT)
            self._GPIO.output(pin, False)

    
    def print_state(self):
        print("Current x = {} ".format(self._x))
        print("Current dir = {} ".format(self._dir))
        print("Current delay = {}\n".format(self._delay))

    def update_input(self,dir=None,delay=None,duration=None):
        if delay is None:
            delay = MIN_DELAY
        if duration is None:
            duration = MAX_DURATION
        if dir is None:
            dir = 1
        with self._writing_lock:
            self._dir = dir
            self._delay = delay
            self._duration = duration

    def force(self,force,duration=None):
        '''
        calculo auxiliar y = ax +b where y:force x:delay
        for force=20 --> delay=0.001
        for force=1  --> delay=1
        result --> x = -0.05257895 y + 1.05257895
        '''        
        if self._enable_commands == True:
            if force>1:
                force =1
            if force<-1:
                force = -1
            # delay = -0.005210526315789473*abs(force) + 0.10521052631578946
            # delay = -0.05257895 * abs(force) + 1.05257895
            delay = 0.001/force/force
            if delay>MAX_DELAY:
                delay = MAX_DELAY
            dir = int(np.sign(force))
            print("entered force = {}".format(force))
            if dir !=0:
                self._disable_writing()
                self.go(dir,delay,duration)
    

    def go(self,dir=None,delay=None,duration=None):
        self.update_input(dir,delay,duration)
        self.print_state()
        if self._running is False:
            self._running = True
            self._task = threading.Thread(target=self._loop)
            self._task.start()

    def _loop(self):
        while not self._stop():
            dir = self._dir
            self._step(dir)
            self._wait()

    def _stop(self):
        if self._duration<=0 or (self._x>=500 and self._dir>0) or (self._x<=-500 and self._dir<0):
            print("Thread stopped")
            self.print_state()
            self._running = False
            self._enable_writing()
            return True
        else:
            return False
    
    def _wait(self,delay=None,check=True):
        ''' wait until self._dir is changed or for the duration of self._delay 
            variable CHECK_DELAY must be a divisor of self._delay for accuracy'''
        if delay is not None:
            self._delay = delay
        sum_delays=0
        init_dir = self._dir
        if check is True:
            while sum_delays<self._delay:
                time.sleep(CHECK_DELAY)
                sum_delays +=CHECK_DELAY
                self._enable_writing()
                if init_dir!=self._dir:
                    break #direction change
            self._prev_delay = sum_delays
        else:
            time.sleep(self._delay)
            self._prev_delay = self._delay
            self._enable_writing()
        self._duration -= self._delay
        
    def _step(self,dir):
        self._StepCounter += dir
        self._x +=dir
        # If we reach the end of the sequence
        # start again
        if (self._StepCounter>=self._StepCount):
            self._StepCounter = 0
        if (self._StepCounter<0):
            self._StepCounter = self._StepCount+dir
        # print("Current position x=",self._x)
        # print(self._Seq[self._StepCounter])
        for pin in range(0, 4):
            xpin = self._StepPins[pin]
            if self._Seq[self._StepCounter][pin]!=0:
                # print(" Enable GPIO %i" %(xpin))
                self._GPIO.output(xpin, True)
            else:
                self._GPIO.output(xpin, False)
    
    def _enable_writing(self):
        with self._writing_lock:
            self._enable_commands = True
            print("writing enabled")
    
    def _disable_writing(self):
        with self._writing_lock:
            self._enable_commands = False
            print("writing disabled")

    def steps(self,steps,delay=None):
        # print("MIN_DELAY: {}, self._delay={}".format(MIN_DELAY,self._delay))
        if steps < 0:
            dir = -1
        else:
            dir = 1
        self.update_input(dir,delay)
        for i in range(abs(steps)):
            self._step(dir)
            self._wait(delay,False)

    def goto(self,x,delay=None):
        self.steps(x-self._x,delay)
    

        

class PanTilt(object):
    def __init__(self, steppers=None,*args, **kwargs):
        self._x = np.array([0.0,0.0])
        self._a = np.array([0.0,0.0])
        self._v = np.array([0.0,0.0])
        self._f = np.array([0.0,0.0])
        self._t = time.time()
        self._steppers = None
        self.set_steppers(steppers)

    def shutdown(self):
        if self._steppers is not None:
            self._steppers[0].shutdown()
            self._steppers[1].shutdown()

    def set_steppers(self,steppers):
        if steppers is None:
            self._steppers = None
        else:
            if isinstance(steppers[0],Stepper) and isinstance(steppers[1],Stepper):
                self._steppers = steppers
            else:
                raise TypeError("stepper should be of type Stepper")

    def move(self,force,angle,duration=None):
        self._f[0] = -cos(radians(angle))*force
        self._f[1] = -sin(radians(angle))*force
        if self._steppers is not None:
            self._steppers[0].force(self._f[0],duration)
            self._steppers[1].force(self._f[1],duration)
    
    def gamepad_move(self,forcex,forcey,duration=None):
        self._f[0]=forcex
        self._f[1]=forcey
        if self._steppers is not None:
            if forcex !=0:
                self._steppers[0].force(self._f[0],duration)
            if forcey !=0:
                self._steppers[1].force(self._f[1],duration)

        
    
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

    code.interact(local=dict(globals(), **locals()))

#calculo auxiliar y = ax +b where y:force x:delay
# for force=20 --> delay=0.001
# for force=1  --> delay=1
# result --> x = -0.05257895 y + 1.05257895
# import numpy as np
# from numpy.linalg import inv
# A = np.array([[0.1,1],[0.001,1]])
# B = np.array([[1],[20]])
# [a,b]=np.matmul(inv(A),B)
# C = inv(np.array([[a[0],b[0]],[0,1]]))
# print("delay = {}*abs(force) + {}".format(C[0,0],C[0,1]))



if __name__ == "__main__":
    main()      

