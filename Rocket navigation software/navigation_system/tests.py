# -*- coding: utf-8 -*-
"""
Created on Sat Feb 26 15:25:40 2022

@author: Thomas Maynadié
"""

import matplotlib.pyplot as plt
import numpy as np

import prefilters.atmospheric_model
from prefilters.pressure_prefilter import Pressure_prefilter, Altitude_simulation

pressure_std_deviation = 0.15
pressure_bias = 0.1
gps_std_deviation = np.array([2.5, 2.5, 2.6])

z_0 = 0
P_0 = 101348

initial_attitude = np.array([0, -80, 0])
az = 2
tf = 10
n = 100

accelerometer_std_deviation = np.array([0.05, 0.051, 0.06])
accelerometer_bias = np.array([0.12, -0.15, 0.08])

sim = Altitude_simulation(pressure_std_deviation, pressure_bias, gps_std_deviation, z_0, P_0, initial_attitude, az, tf, n)

x_est = [np.array([z_0, 0, az, pressure_bias])]
t_est = np.linspace(0, tf, n)

prefilter = Pressure_prefilter(pressure_std_deviation, pressure_bias, gps_std_deviation, accelerometer_std_deviation, accelerometer_bias, z_0, P_0, initial_attitude)

for i in range(1, n):
    dt = t_est[i] - t_est[i-1]
    
    measurement = sim.get_measurements(accelerometer_bias, accelerometer_std_deviation, i)
    input_vector = 0
    
    if i > 1220: x_est.append(prefilter.output(input_vector, measurement, dt, accelerometer_bias, accelerometer_std_deviation, False, True))
    else: x_est.append(prefilter.output(input_vector, measurement, dt, accelerometer_bias, accelerometer_std_deviation, True, True))
        
x_true = sim.get_state_vector()
x_est = np.array(x_est).transpose()

fig = plt.figure(dpi = 400)
axs = fig.subplots(len(x_est), 1)

for i in range(len(x_est)):
    axs[i].plot(t_est, x_true[i])
    axs[i].plot(t_est, x_est[i])
    
print('ok')