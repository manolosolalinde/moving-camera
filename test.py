from driver.stepper_driver import *
import code

pan = Stepper(PINS_PAN)
tilt = Stepper(PINS_TILT)
steppers = (pan,tilt)
pantilt = PanTilt(steppers)

code.interact(local=dict(globals(), **locals()))