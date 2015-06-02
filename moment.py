from visual import *
from math import *
from numpy import linspace

#position, mass
mass_cloud = [
        #motors
        [vector(5,0,5),3],
        [vector(5,0,-5),3],
        [vector(-5,0,5),3],
        [vector(-5,0,-5),3],
        #body
        [vector(3,0,0),6],
        [vector(-3,0,0),10],

        #camera
        [vector(3,-3,0),4],
    ]

def calc_centroid():
    centroid = vector()
    total_mass = 0.0
    for (pt_pos, pt_mass) in mass_cloud:
        centroid += pt_pos*pt_mass
        total_mass += pt_mass

    centroid /= total_mass
    return centroid

def move_centroid_to_zero():
    centroid = calc_centroid()
    for i in range(len(mass_cloud)):
        mass_cloud[i][0] -= centroid

def point_distance(pt, axis):
    return ((pt).cross((pt-axis))).mag/axis.mag

def moment_of_inertia(axis):
    moment = 0.0
    for (pt_pos,pt_mass) in mass_cloud:
        moment += pt_mass * point_distance(pt_pos,axis)**2
    return moment

scene.userspin = True
scene.userzoom = True

move_centroid_to_zero()

for (pt_pos,pt_mass) in mass_cloud:
    sphere(pos=pt_pos,radius=pt_mass**1.0/3.0*0.62,color=color.red)

def copter_tilt_torque(x,y):
    #this function could be far simpler
    m1_roll_factor = -1/sqrt(2)
    m2_roll_factor = 1/sqrt(2)
    m3_roll_factor = 1/sqrt(2)
    m4_roll_factor = -1/sqrt(2)
    m1_pitch_factor = 1/sqrt(2)
    m2_pitch_factor = -1/sqrt(2)
    m3_pitch_factor = 1/sqrt(2)
    m4_pitch_factor = -1/sqrt(2)
    m_force = [0,0,0,0]
    m_force[0] = m1_roll_factor*x + m1_pitch_factor*y
    m_force[1] = m2_roll_factor*x + m2_pitch_factor*y
    m_force[2] = m3_roll_factor*x + m3_pitch_factor*y
    m_force[3] = m4_roll_factor*x + m4_pitch_factor*y

    factor = (max(m_force)-min(m_force))/2
    for i in range(len(m_force)):
        m_force[i]/=factor

    roll_torque = 0.5*(m_force[0]*m1_roll_factor+m_force[1]*m2_roll_factor+m_force[2]*m3_roll_factor+m_force[3]*m4_roll_factor)
    pitch_torque = 0.5*(m_force[0]*m1_pitch_factor+m_force[1]*m2_pitch_factor+m_force[2]*m3_pitch_factor+m_force[3]*m4_pitch_factor)
    return sqrt(roll_torque**2+pitch_torque**2)

moment_scale = 1.0e-1
torque_scale = 15

for lat in [0]+list(linspace(-pi,pi,150)):
    cos_lat = cos(lat)
    for lon in linspace(0,2*pi,150*cos_lat):
        x = cos(lon) * cos_lat
        y = sin(lon) * cos_lat
        z = sin(lat)
        axis = vector(x,z,y)

        moment_capability_in_axis = moment_of_inertia(axis)
        moment_plot = axis*moment_capability_in_axis*moment_scale

        if z != 0:
            sphere(pos=moment_plot,radius=.5,color=color.blue,opacity=0.2)
        else:
            torque_capability_in_axis = copter_tilt_torque(axis.x,axis.z)
            torque_plot = axis*torque_capability_in_axis*torque_scale
            accel_plot = axis*(torque_capability_in_axis/moment_capability_in_axis)*2.5e4

            sphere(pos=moment_plot,radius=.5,color=color.blue,opacity=1.0)
            sphere(pos=torque_plot,radius=.5,color=color.yellow)
            sphere(pos=accel_plot,radius=.5,color=color.green)

while True:
    rate(60)
