#!/usr/bin/python
from Model import *
from Data import data
from numpy import linspace
import matplotlib.pyplot as plt
from pymavlink import mavutil
from scipy.optimize import minimize
import sys

def differentiate(array,dt):
    array = [array[0]]+array+[array[len(array)-1]]
    ret = []
    for i in range(1,len(array)-1):
        ret.append(((array[i+1] - array[i]) + (array[i] - array[i-1]))/(2*dt))
    return ret

gyro_x = []
gyro_y = []
gyro_z = []
mot1 = []
mot2 = []
mot3 = []
mot4 = []
g_times = []
r_times = []

r_t = 0
g_t = 0
dt = 0.0025

for i in data:
    if i[0] == 'g':
        g_times.append(g_t)
        gyro_x.append(i[1])
        gyro_y.append(i[2])
        gyro_z.append(i[3])
        g_t += dt
    if i[0] == 'r':
        r_times.append(r_t)
        mot1.append(i[1])
        mot2.append(i[2])
        mot3.append(i[3])
        mot4.append(i[4])
        r_t += dt

if len(r_times) > len(g_times):
    r_times = r_times[:len(g_times)]
    mot1 = mot1[:len(g_times)]
    mot2 = mot2[:len(g_times)]
    mot3 = mot3[:len(g_times)]
    mot4 = mot4[:len(g_times)]

if len(g_times) > len(r_times):
    g_times = g_times[:len(r_times)]
    gyro_x = gyro_x[:len(g_times)]
    gyro_y = gyro_y[:len(g_times)]
    gyro_z = gyro_z[:len(g_times)]

ang_acc_x = differentiate(gyro_x,0.0025)
ang_acc_y = differentiate(gyro_y,0.0025)
ang_acc_z = differentiate(gyro_z,0.0025)

model_ang_acc_x = []
model_ang_acc_y = []
model_ang_acc_z = []

c = Copter(0.0025, 7.51310894,  3.59830976,  0.8518829 ,  0.06937893,  0.03314607,
        0.0248389 ,  0.01776458, -0.02361771,  0.02671136,  0.48564235,
        0.54183854)

for i in range(len(mot1)):
    c.update(mot1[i]-1117,mot2[i]-1117,mot3[i]-1117,mot4[i]-1117,0.0025)
    model_ang_acc_x.append(c.omega_dot_x)
    model_ang_acc_y.append(c.omega_dot_y)
    model_ang_acc_z.append(c.omega_dot_z)

plt.figure(1)
plt.subplot(311)
plt.plot(r_times,model_ang_acc_x,r_times,ang_acc_x)
plt.subplot(312)
plt.plot(r_times,model_ang_acc_y,r_times,ang_acc_y)
plt.subplot(313)
plt.plot(r_times,model_ang_acc_z,r_times,ang_acc_z)
plt.show()
sys.exit(0)

def error_func(params):
    print params
    sq_resid = 0
    c = Copter(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], params[8])
    for i in range(len(mot1)):
        c.update(mot1[i]-1117,mot2[i]-1117,mot3[i]-1117,mot4[i]-1117,0.0025)
        resid_x = (c.omega_dot_x-ang_acc_x[i])
        resid_y = (c.omega_dot_y-ang_acc_y[i])
        resid_z = (c.omega_dot_z-ang_acc_z[i])
        sq_resid += (resid_x**2+resid_y**2+resid_z**2)

    sq_resid /= len(mot1)
    if isnan(sq_resid) or isinf(sq_resid):
        sq_resid = sys.float_info.max
    print sq_resid
    return sq_resid

params = [1, 1, 1, 1, .1, .1, .1, 0.0, .02]
bounds = [(.001,100.0), (.001,100.0), (.1,100.0), (.001,100.0), (.01,10.0), (.01,10.0), (.0001,10.0), (-.1,.1), (0.0,0.0)]

print minimize(error_func, params, bounds=bounds)

print len(r_times), len(g_times), len(model_ang_acc_x),len(ang_acc_x)


