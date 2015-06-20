from math import *
from collections import deque
class Motor:
    def __init__(self, Ki, Ktr, Im, Kd):
        self.Ki = Ki # back-emf constant of motor
        self.Ktr = Ktr # torque constant of motor/resistance of motor
        self.Im = Im # moment of inertia of prop-motor system
        self.Kd = Kd # 0.5*p*Cd*A
        self.omega = 0.0 # ang vel of prop-motor system
        self.t_motor = 0.0

    def update(self,volts_in,dt):
        V_motor = max(0,volts_in - self.Ki*self.omega)
        self.t_motor = self.Ktr * V_motor
        omega_dot = (self.t_motor-self.Kd * self.omega**2) / self.Im
        self.omega += omega_dot*dt

    def get_ang_vel(self):
        return self.omega

    def get_torque(self):
        return self.t_motor

class DelayModel:
    def __init__(self, delay_samples, alpha):
        self.delay_samples = delay_samples
        self.alpha = alpha
        self.hist=deque()
        self.filtered = 0

    def apply(self, val):
        self.filtered += (val-self.filtered)*self.alpha

        if self.delay_samples == 0:
            return self.filtered

        self.hist.append(self.filtered)

        if len(self.hist) > ceil(self.delay_samples)+1:
            self.hist.popleft()

        if len(self.hist) < ceil(self.delay_samples) or len(self.hist)==1:
            return self.hist[0]
        else:
            factor = self.delay_samples-floor(self.delay_samples)
            return self.hist[0]*factor+self.hist[1]*(1.0-factor)

'''
CW    CCW
  3   1
    X
  2   4
CCW    CW
'''

class Copter:
    def __init__(self, dt, Ki, Ktr, Im, Kd, x_gain, y_gain, z_gain, yaw_misalignment, system_lag, cg_x, cg_y, k_drag_x, k_drag_y, k_drag_z, x_thr_gain, y_thr_gain, z_thr_gain):
        self.m1 = Motor(Ki, Ktr, Im, Kd)
        self.m2 = Motor(Ki, Ktr, Im, Kd)
        self.m3 = Motor(Ki, Ktr, Im, Kd)
        self.m4 = Motor(Ki, Ktr, Im, Kd)
        self.x_gain = x_gain
        self.y_gain = y_gain
        self.z_gain = z_gain
        self.cg_x = cg_x
        self.cg_y = cg_y
        self.cos_yaw_misalignment = cos(yaw_misalignment)
        self.sin_yaw_misalignment = sin(yaw_misalignment)
        self.omega_dot_x = 0.0
        self.omega_dot_y = 0.0
        self.omega_dot_z = 0.0
        self.omega_x = 0.0
        self.omega_y = 0.0
        self.omega_z = 0.0
        alpha = dt/(dt+system_lag)
        self.omega_dot_hist_x = DelayModel(0,alpha)
        self.omega_dot_hist_y = DelayModel(0,alpha)
        self.omega_dot_hist_z = DelayModel(0,alpha)

    def update(self,vm1,vm2,vm3,vm4,dt):
        self.m1.update(vm1,dt)
        self.m2.update(vm2,dt)
        self.m3.update(vm3,dt)
        self.m4.update(vm4,dt)

        _omega_dot_x = (self.cg_x*(self.m3.get_ang_vel()**2 + self.m2.get_ang_vel()**2) - (1-self.cg_x)*(self.m1.get_ang_vel()**2 + self.m4.get_ang_vel()**2)) * self.x_gain #roll
        _omega_dot_y = (self.cg_y*(self.m3.get_ang_vel()**2 + self.m1.get_ang_vel()**2) - (1-self.cg_y)*(self.m2.get_ang_vel()**2 + self.m4.get_ang_vel()**2)) * self.y_gain #pitch
        _omega_dot_z = (self.m1.get_torque() + self.m2.get_torque() - self.m3.get_torque() - self.m4.get_torque()) * self.z_gain #yaw

        self.omega_dot_x = self.omega_dot_hist_x.apply(_omega_dot_x * self.cos_yaw_misalignment - _omega_dot_y * self.sin_yaw_misalignment)
        self.omega_dot_y = self.omega_dot_hist_y.apply(_omega_dot_x * self.sin_yaw_misalignment + _omega_dot_y * self.cos_yaw_misalignment)
        self.omega_dot_z = self.omega_dot_hist_z.apply(_omega_dot_z)

        self.omega_x += self.omega_dot_x * dt
        self.omega_y += self.omega_dot_y * dt
        self.omega_z += self.omega_dot_z * dt        

        self.theta_x = self.omega_x*dt + self.omega_dot_x*dt**2
        self.theta_y = self.omega_y*dt + self.omega_dot_y*dt**2
        self.theta_z = self.omega_z*dt + self.omega_dot_z*dt**2

        total_thrust = self.m1.get_ang_vel()**2 + self.m2.get_ang_vel()**2 + self.m3.get_ang_vel()**2 + self.m4.get_ang_vel()**2
        a_tx = total_thrust * sin(self.theta_z) * sin(self.theta_y) * self.x_thr_gain
        a_ty = total_thrust * sin(self.theta_z) * sin(self.theta_x) * self.y_thr_gain
        a_tz = total_thrust * sim(self.theta_x) * sin(self.theta_y) * self.z_thr_gain

        #(v_wx, v_wy, v_wz) = to_body_frame(Vwn, Vwe, Vwd, self.theta_x, self.theta_y, self.theta_z)

        ins_vel_x = (a_tx*dt) + self.vel_x #+ v_wx
        ins_vel_y = (a_ty*dt) + self.vel_y #+ v_wy
        ins_vel_z = (a_tz*dt) + self.vel_z #+ v_wz

        a_dx = (sqrt(4*k_drag_x*dt*ins_vel_x + 1) + 2*k*dt*ins_vel_x + 1)/(2*k_drag_x * dt**2)
        a_dy = (sqrt(4*k_drag_y*dt*ins_vel_y + 1) + 2*k*dt*ins_vel_y + 1)/(2*k_drag_y * dt**2)
        a_dz = (sqrt(4*k_drag_z*dt*ins_vel_z + 1) + 2*k*dt*ins_vel_z + 1)/(2*k_drag_z * dt**2)

        self.accel_x = a_tx - a_dx
        self.accel_y = a_ty - a_dy
        self.accel_z = a_tz - a_dz
