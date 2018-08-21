import numpy as np
import time

#usage: pan = Servo(0)
# pan.goto(x1) move servo to position x1

class Servo:
    def __init__(self, id=0, init_a=0, init_v=0, init_x=0):
        self_id = id # 0 for pan 1 for tilt
        self._a = init_a # acceleration
        self._v = init_v # velocity
        self._x = init_x # current position
        self._xf = 0 # position we are going
        self._epsilon = 0.01 #small constant
        self._t = time.time() #current time
        self.set_parameters()

    def set_parameters(self,max_acceleration=1000,max_velocity=150,max_x=100):
        self._a_max = max_acceleration
        self._v_max = max_velocity
        self._x_max = max_x
        self._dx_max = self._v_max**2/2/self._a_max
        self._dx_epsilon = self._epsilon * max_x 
        self._dv_epsilon = self._epsilon * max_x * 10 

    def _stop_check_x(self):
        if abs(self._xf-self._x)<self._dx_epsilon and abs(self._v)<self._dv_epsilon:
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
                    self._a = -self._a_max * sign_dx
        
        # Update velocity
        self._v = self._v + self._a * dt

        # Update x
        self._x = self._x + self._v * dt + self._a/2*dt*dt
        
    def goto(self,x1):
        self._xf = x1
        self._t = time.time()
        while not self._stop_check_x() :
            time.sleep(0.01)
            prev_t = self._t
            self._t = time.time()
            dt = self._t - prev_t
            self._update(dt)
            #print(f"Location: {self._x}, velocity: {self._v}, acceleration: {self._a}, dt: {dt}")
            print("Location: {}, velocity: {}, acceleration: {}, dt: {}".format(self._x,self._v,self._a,dt))

def main():
    pan = Servo(0)
    pan.goto(-60)

if __name__ == "__main__":
    main()
