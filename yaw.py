from Model import Motor
import matplotlib.pyplot as plt
from random import gauss
from scipy.fftpack import fft
from scipy import signal
import numpy as np
from math import *

m = Motor(7.51310894,  3.59830976,  0.8518829 ,  0.06937893)
m2 = Motor(7.51310894,  3.59830976,  0.8518829 ,  0.06937893)

t = -10
dt = 0.0025

x = []
y = []
y2 = []

volts = 300

steady_thrust = 0

while t < 2:
    if t < .025:
        m.update(330,dt)
        m2.update(330,dt)
    else:
        m.update(330-50,dt)
        m2.update(330+51.2,dt)
    if t < 0:
        t += dt
        steady_thrust = m.get_ang_vel()
        continue

    x.append(t)
    y.append(-m.get_ang_vel()+2*steady_thrust)
    y2.append(m2.get_ang_vel())
    t+=dt


plt.plot(x,y,x,y2)
plt.show()
