"""
Show how to connect to keypress events
"""
from __future__ import print_function
import sys
import numpy as np
import matplotlib.pyplot as plt

x = np.array([0,0])

def press(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'x':
        visible = xl.get_visible()
        xl.set_visible(not visible)
    if event.key == 'up':
        x[1]+=1
    if event.key == 'down':
        x[1]+=-1
    if event.key == 'right':
        x[0]+=1
    if event.key == 'left':
        x[0]+=-1
    ax.cla()
    ax.set_ylim(-10,10)
    ax.set_xlim(-10,10)
    ax.plot(x[0],x[1], 'go')
    fig.canvas.draw()
    
fig, ax = plt.subplots()


fig.canvas.mpl_connect('key_press_event', press)
xl = ax.set_xlabel('easy come, easy go')
ax.set_title('Press a key')
ax.set_autoscale_on(False)
ax.set_ylim(-10,10)
ax.set_xlim(-10,10)
ax.plot(x[0],x[1], 'go')
plt.show()
