# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 17:09:39 2022

@author: Thomas Maynadié
"""

import numpy as np
from ..quaternion_helper import quaternion
from .. import frames

class rotation_sim_quaternions():
    def __init__(self, init_attitude, w, accel_enu, tf, N, r_ecef, frame='ecef'):
        self.q = quaternion(*init_attitude)
        self.w = w
        if frame == 'ecef':
            self.w = frames.ecef_to_enu(self.w, r_ecef, only_rotation=True)

        self.q_traj = [list(self.q.get_coefficients())]
        self.t = [0]

        self.accel_enu = accel_enu
        
        dt = tf/N
        
        qi = quaternion(*self.q.get_coefficients())
        print(self.w.shape)
        
        for i in range(1, N):
            if isinstance(w[0], np.ndarray):
                qi = self.__compute_attitude_estimate(w[:,i], qi, dt)
            else:
                qi = self.__compute_attitude_estimate(w, qi, dt)

            self.q_traj.append(list(qi.get_coefficients()))
            self.t.append(self.t[i-1] + dt)
            
    def get_trajectory(self):
        return np.array(self.t), np.array(self.q_traj)
    
    def measurement(self, s_a1, s_a2, b_a1, b_a2, s_w1, s_w2, b_w1, b_w2, s_mag1, s_mag2, b_mag1, b_mag2, mag_declination, n):
        M = quaternion(*self.q_traj[n]).DCM().T # compute ENU to RF rotation matrix

        if isinstance(self.w[0], np.ndarray):
            angular_rate = self.w[:,n]
            
            w_meas_1 = angular_rate + b_w1 + np.random.normal(np.zeros(3), s_w1)
            w_meas_2 = angular_rate + b_w2 + np.random.normal(np.zeros(3), s_w2)
        else:
            angular_rate = self.w
                
            w_meas_1 = angular_rate + b_w1 + np.random.normal(np.zeros(3), s_w1)
            w_meas_2 = angular_rate + b_w2 + np.random.normal(np.zeros(3), s_w2)
        
        q0, q1, q2, q3 = self.q_traj[n]
        
        mx_enu = np.sin(mag_declination)
        my_enu = np.cos(mag_declination)
        mz_enu = 0

        B_enu = np.array([mx_enu, my_enu, mz_enu])
        g = np.array([0, 0, 1])
        
        B_rf = np.dot(M, B_enu)
        g_rf = np.dot(M, g)
        
        B_meas_1 = B_rf + b_mag1 + np.random.normal(np.zeros(3), s_mag1, 3)
        B_meas_2 = B_rf + b_mag2 + np.random.normal(np.zeros(3), s_mag2, 3)

        a_meas_1 = g_rf + b_a1 + np.random.normal(np.zeros(3), s_a1, 3)
        a_meas_2 = g_rf + b_a2 + np.random.normal(np.zeros(3), s_a2, 3)

        pressure = 101325.0
        gps = np.zeros(6)

        return a_meas_1, a_meas_2, w_meas_1, w_meas_2, B_meas_1, B_meas_2, pressure, gps
        
    
    def __compute_attitude_estimate(self, angular_rate, q, dt):                
        q_new = q.get_coefficients()

        wx, wy, wz = angular_rate
        q0, q1, q2, q3 = q_new
        q_new[0] += (-q1*wx - q2*wy - q3*wz) * dt/2
        q_new[1] += ( q0*wx - q3*wy + q2*wz) * dt/2
        q_new[2] += ( q3*wx + q0*wy - q1*wz) * dt/2
        q_new[3] += (-q2*wx + q1*wy + q0*wz) * dt/2
        
        normalization_factor = 1 / np.sqrt(np.sum(q_new**2))
        q_new = quaternion(*(q_new * normalization_factor))
        
        return q_new