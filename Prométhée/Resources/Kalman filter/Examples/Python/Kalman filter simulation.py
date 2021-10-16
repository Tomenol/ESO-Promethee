# -*- coding: utf-8 -*-
"""
Created on Sun Oct 10 10:10:59 2021

Script to compare simulated position with estimated using a kalman filter

@author: Thomas Maynadié
"""

import numpy as np
import matplotlib.pyplot as plt

def kalman_gps(z_gps, z_pressure, a, x_hat, P_hat, dt, sigma_a, sigma_pressure, sigma_gps):    
    """Used to compute the estimated state vector (Kalman) using IMU + GPS + pressure measurements
    
    Inputs :
        - z_gps : GPS measurements
        - z_pressure : pressure sensor measurements
        - a : accelerometer measurements
        - x_hat : actual state vector
        - P_hat : actual estimate covariance matrix
        - dt : time step between two estimates
        - sigma_a : accelerometer measurements standard deviation
        - sigma_pressure : pressure measurements standard deviation
        - sigma_gps : GPS measurements standard deviation
        
    Returns :
        - x_hat : estimated state vector
        - P_hat : updated estimate covariance matrix
    """
    n = np.size(x_hat)
    
    z = np.matrix([z_gps, z_pressure]).T
    u = np.matrix([a]).T

    F = np.matrix([[1, dt, 0, 0], 
                   [0, 1, -dt, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    
    B = np.matrix([0, dt, 0, 0]).T
    
    R = np.diag([sigma_gps**2, sigma_pressure**2])
    H = np.matrix([[1, 0, 0, 0], 
                   [1, 0, 0, 1]])

    Q = B.T*B * sigma_a ** 2
    
    # predict
    x_prediction = F*x_hat + B * u
    P_prediction = F*P_hat*F.T + Q
    
    del Q, B, u
    
    # update
    S = H * P_prediction * H.T + R
    K = P_prediction * H.T * np.linalg.inv(S)
    
    del S, R
    
    x_hat = x_prediction + K * (z - H * x_prediction)
    P_hat = (np.eye(n) - K*H) * P_prediction
    
    return x_hat, P_hat

def kalman_no_gps(a, z_pressure, x_hat, P_hat, dt, sigma_a, sigma_pressure):    
    """Used to compute the estimated state vector (Kalman) using only pressure + IMU measurements
    
    Inputs :
        - a : accelerometer measurements
        - z_pressure : accelerometer measurements
        - x_hat : actual state vector
        - P_hat : actual estimate covariance matrix
        - dt : time step between two estimates
        - sigma_a : accelerometer measurements standard deviation
        - sigma_pressure : pressure measurements standard deviation
        
    Returns :
        - x_hat : updated state vector
        - P_hat : updated estimate covariance matrix
    """
    n = np.size(x_hat)
    
    z = np.matrix([z_pressure]).T
    u = np.matrix([a])

    F = np.matrix([[1, dt, 0, 0], 
                   [0, 1, -dt, 0],
                   [0, 0, 1, 0], 
                   [0, 0, 0, 1]])
    
    B = np.matrix([0, dt, 0, 0]).T
    
    R = np.diag([sigma_pressure**2])
    H = np.matrix([1, 0, 0, 1])

    Q = B.T*B * sigma_a ** 2
    
    # predict
    x_prediction = F*x_hat + B * u
    P_prediction = F*P_hat*F.T + Q
    
    # update
    S = H * P_prediction * H.T + R
    K = P_prediction * H.T * np.linalg.inv(S)
    
    x_hat = x_prediction + K * (z - H * x_prediction)
    P_hat = (np.eye(n) - K*H) * P_prediction
    
    return float(x_hat[0]), float(x_hat[1])

def main():
    # setup time interval
    t_start = 0
    t_final = 100
    
    # setup measurement properties
    F_e = 10
    Te = 1/F_e
    N_e = int(F_e * (t_final - t_start)) + 1
    
    # creates an arbitrary trajectory along the x-axis
    t = np.linspace(t_start, t_final, N_e)
    
    a = 0.001
    b = 1
    
    x_real = a*(-b * t ** 3 * (t - (t_final - t_start)))          # real position
    v_real = a*(-4 * b * t ** 3 + 3 * (t_final - t_start) * t ** 2)   # real velocity
    a_real = a*(-12 * b * t ** 2 + 6 * (t_final - t_start) * t)      # real acceleration
    
    #compute measurements associated with the real trajectory
    sigma_gps = 5
    sigma_pressure = 10
    sigma_a = 0.1
    
    b_accel1 = 0.12*(np.exp(1.254e-2*t)-1)
    b_accel2 = 0.1*(np.exp(1e-2*t)-1)
    a_accel_1 = a_real + (b_accel1 + np.random.normal(0, sigma_a, N_e)) * 9.81
    a_accel_2 = a_real + (b_accel2 + np.random.normal(0, sigma_a, N_e)) * 9.81
    
    x_gps = np.floor(x_real) + np.random.normal(0, sigma_gps, N_e)
    x_pressure = x_real + 10 + np.random.normal(0, sigma_pressure, N_e)
    
    # setup Kalman filter variables
    last_t = 0
    last_ax_f = 0
    last_pos_accelerometer_only = 0
    last_vel_accelerometer_only = 0
    
    pos_kalman = []
    pos_accelerometer_only = []
    
    x_hat_1 = np.matrix(np.zeros(4)).T
    x_hat_2 = np.matrix(np.zeros(4)).T
    
    P_hat_1 = np.matrix(np.zeros((4, 4)))
    P_hat_2 = np.matrix(np.zeros((4, 4)))
    
    # compute Kalman filter state vector estimates 
    for i in range(len(a_accel_1)):
        dt = t[i] - last_t
        last_t = t[i]
        
        # compute position without using the kalman filter
        last_vel_accelerometer_only += a_accel_1[i] * dt
        last_pos_accelerometer_only += last_vel_accelerometer_only * dt
        
        # compute position using the kalman filter
        x_hat_1, P_hat_1 = kalman_gps(x_gps[i], x_pressure[i], a_accel_1[i], x_hat_1, P_hat_1, dt, sigma_a, sigma_pressure, sigma_gps)
        x_hat_2, P_hat_2 = kalman_gps(x_gps[i], x_pressure[i], a_accel_2[i], x_hat_2, P_hat_2, dt, sigma_a, sigma_pressure, sigma_gps)

        pos_kalman.append(0.5*(float(x_hat_1[0]+float(x_hat_2[0]))))
        pos_accelerometer_only.append(last_pos_accelerometer_only)
       
    # plot results :
    fig1 = plt.figure(dpi=300)
    axis1 = fig1.add_subplot(111)
    axis1.set_title("position")
    axis1.grid()
    
    axis1.plot(t, x_real)
    axis1.plot(t, x_gps)
    axis1.plot(t, pos_accelerometer_only)
    axis1.plot(t, x_pressure)
    axis1.plot(t, pos_kalman)
    axis1.legend(["real", "GPS only", "Accel only", "Pressure only", "Kalman"])

    # plot results
    fig2 = plt.figure(dpi=300)
    axis2 = fig2.add_subplot(111)
    axis2.set_title("Acceleration")
    axis2.grid()
    
    axis2.plot(t, a_real/9.81)
    axis2.plot(t, a_accel_1/9.81)
    
    axis2.legend(["real", "accelerometer"])
    
    # plot results
    fig1 = plt.figure(dpi=300)
    axis1 = fig1.add_subplot(111)
    axis1.set_title("Position error (m)")
    axis1.grid()
    
    axis1.plot(t, (pos_accelerometer_only - x_real))
    axis1.plot(t, (x_gps - x_real))
    axis1.plot(t, (x_pressure - x_real))
    axis1.plot(t, (pos_kalman - x_real))
    
    axis1.legend(["Accel only", "GPS only", "Pressure only", "Kalman"])
    
    # plot results
    fig1 = plt.figure(dpi=300)
    axis1 = fig1.add_subplot(111)
    axis1.set_title("Position error (m)")
    axis1.grid()
    
    axis1.plot(t, (x_pressure - x_real), ":", linewidth=.7)
    axis1.plot(t, (x_gps - x_real), linewidth=.7)
    axis1.plot(t, (pos_kalman - x_real), linewidth=1)
    
    axis1.legend(["Pressure only", "GPS only", "Kalman"])

if __name__ == '__main__':
    main()