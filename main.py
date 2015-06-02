#!/usr/bin/python

# rms residuals with...
    # all parameters fit: 7.29873837242 (0%)
    # latency disabled: 7.39941108822 (1.38%)
    # lag disabled: 8.28967798218 (13.58%)
    # latency and lag disabled: 9.10414960889 (24.74%)
    # no yaw misalignment: 7.34976598951 (.69%)
    # no batt impedence: 7.29873837454 (0%)
    # no cg parameters: 9.55460202597 (30.9%)

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

def error_func(params):
    print params
    sq_resid = 0
    c = Copter(0.0025, params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], params[8], params[9], params[10])
    for i in range(len(mot1)):
        c.update(mot1[i]-params[11],mot2[i]-params[11],mot3[i]-params[11],mot4[i]-params[11],0.0025)
        if i > 400*5:
            resid_x = (c.omega_dot_x-ang_acc_x[i])
            resid_y = (c.omega_dot_y-ang_acc_y[i])
            resid_z = (c.omega_dot_z-ang_acc_z[i])
            sq_resid += (resid_x**2+resid_y**2+resid_z**2)

    sq_resid /= len(mot1)
    if isnan(sq_resid) or isinf(sq_resid):
        sq_resid = sys.float_info.max
    print sq_resid
    return sq_resid

params = [7.51810770e+00,   3.58577766e+00,   8.64518317e-01,
         5.87832121e-02,   2.84742353e-02,   2.13692233e-02,
         1.74909129e-02,  -2.35811974e-02,   2.66885902e-02,
         4.86427532e-01,   5.37986428e-01,   1117.0]
bounds = [(.001,100.0), (.001,100.0), (.001,100.0), (.001,100.0), (.01,10.0), (.01,10.0), (.0001,10.0), (-.1,.1), (0.0,0.05), (0.0,1.0),(0.0,1.0), (1000,1150)]

print minimize(error_func, params, bounds=bounds, method='TNC')

print len(r_times), len(g_times), len(model_ang_acc_x),len(ang_acc_x)

plt.plot(r_times,model_ang_acc_x,r_times,ang_acc_x)
plt.show()
