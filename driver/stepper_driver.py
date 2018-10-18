import threading
import time

DELAY = 100 # Default delay in ms

PINS_PAN= [17,22,23,24]
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
    def __init__(self, pins,*args, **kwargs):
        self._delay = 0
        self._duration= 4 #loops
        self._running = False
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
        self._WaitTime = 10/float(1000)
        # Initialise variables
        self._StepCounter = 0

    def _update_input(self,dir=1,delay=None,duration=None):
        if delay is None:
            delay = DELAY
        if duration is None:
            duration = abs(int(1000/delay))
        self._delay = delay
        self._duration = duration

    def start(self,delay=None,duration=None):
        self._update_input(delay,duration)
        if self._running is False:
            self._running = True
            task = threading.Thread(target=self._loop)
            task.start()
    
    def _stop(self):
        if self._duration<=0:
            self._running = False
            return True
        else:
            return False
    
    def _loop(self):
        while not self._stop():
            self._step()

    def _step(self,dir):
        time.sleep(self._delay)
        self._StepCounter += dir
        self._x +=dir
        self._duration+=-1
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
            self._step(dir,delay)