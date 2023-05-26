# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 16:49:59 2021

@author: Thomas Maynadié
"""

import numpy as np
import matplotlib.pyplot as plt

def rotation_matrix(theta):
    R_x = np.matrix([[1,                     0,                  0],
                    [0,                     np.cos(theta[0]), -np.sin(theta[0])],
                    [0,                     np.sin(theta[0]), np.cos(theta[0])]])
    R_y = np.matrix([[np.cos(theta[1]),    0,      np.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-np.sin(theta[1]),   0,      np.cos(theta[1])  ] ])
    R_z = np.matrix([[np.cos(theta[2]),    -np.sin(theta[2]),    0],
                    [np.sin(theta[2]),    np.cos(theta[2]),     0],
                    [0,                     0,                      1]])

    return np.array(np.dot(R_z, np.dot(R_y, R_x)))



# real rotations
r = np.array([0.15, -0.10, 0.20])

t = np.linspace(0, 10, 100)

theta_dot_dot = 0.20 # rad/s
phi_dot_dot = -1.5 # rad/s
psi_dot_dot = 1 # rad/s
alpha = np.array([theta_dot_dot, phi_dot_dot, psi_dot_dot])

theta_dot = theta_dot_dot * t
phi_dot = phi_dot_dot * t
psi_dot = psi_dot_dot * t

R_01 = rotation_matrix([-np.pi/2, 0, np.pi/2])
R_10 = np.linalg.inv(R_01)

v = []
accel = []

accel_x_measurement = []
accel_y_measurement = []
accel_z_measurement = []

angular_vel_x_measurement = []
angular_vel_y_measurement = []
angular_vel_z_measurement = []

previous_angular_vel_measurement = np.array([0, 0, 0])

for i in range(0, 99):
    omega = np.array([theta_dot[i+1], phi_dot[i+1], psi_dot[i+1]])
    
    v.append(np.cross(omega, r))
    accel.append(np.cross(omega, np.cross(omega, r)) + np.cross(alpha, r))
    
    accel_measurement = np.dot(R_01, accel[i])
    angular_vel_measurement = np.dot(R_01, omega)
    
    # transforming to center of gravity
    angular_vel_measurement = np.dot(R_10, angular_vel_measurement)
    accel_measurement = np.dot(R_10, accel_measurement)
    
    estimated_angular_acceleration = 1/0.1*(angular_vel_measurement - previous_angular_vel_measurement)
    previous_angular_vel_measurement = angular_vel_measurement
    
    print(estimated_angular_acceleration)
    
    accel_measurement = accel_measurement - np.cross(angular_vel_measurement, np.cross(angular_vel_measurement, r)) - np.cross(estimated_angular_acceleration, r)
    
    accel_x_measurement.append(accel_measurement[0])
    accel_y_measurement.append(accel_measurement[1])
    accel_z_measurement.append(accel_measurement[2])
    angular_vel_x_measurement.append(angular_vel_measurement[0])
    angular_vel_y_measurement.append(angular_vel_measurement[1])
    angular_vel_z_measurement.append(angular_vel_measurement[2])

plt.plot(t[1:100], accel_x_measurement)
