# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 09:10:49 2022

@author: Thomas Maynadié
"""

import numpy as np

from ..helpers.quaternion_helper import quaternion
from ..helpers.simulation.rotation_simulation import rotation_sim_quaternions

class NavigationEKF:
    def __init__(self, initial_attitude, initial_position, gyroscope_std_deviation, gyroscope_bias, accelerometer_std_deviation, accelerometer_bias, magnetometer_std_deviation, magnetometer_bias, gps_pos_std_deviation, gps_vel_std_deviation, pressure_std_deviation, pressure_bias, temperature_std_deviation, temperature_bias, mag_declination):
        # measurement errors
        self.gyroscope_std_deviation = gyroscope_std_deviation
        self.gyroscope_bias = gyroscope_bias

        self.accelerometer_std_deviation = accelerometer_std_deviation
        self.accelerometer_bias = accelerometer_bias
        
        self.magnetometer_std_deviation = magnetometer_std_deviation
        self.magnetometer_bias = magnetometer_bias

        # state vector x = [q0, q1, q2, q3, bwx, bwy, bwz, b_ax, b_ay, b_az, b_mx, b_my, b_mz,  ].T
        # EKF parameters
        self.state_space_dim = 13
        self.measurement_space_dim = 6
        
        # initialize estimate covariance matrix P
        self.P = np.diag([1e-6, 1e-6, 1e-6, 1e-6, *self.gyroscope_bias**2, *self.accelerometer_bias**2, *self.magnetometer_bias**2])
        
        # initialize state vector x
        self.x = np.zeros(self.state_space_dim)
        
        self.initial_quaternion = quaternion(*initial_attitude)
        
        self.x[0:4] = self.initial_quaternion.get_coefficients()
        self.x[4:7] = self.gyroscope_bias*0.95
        self.x[7:10] = self.accelerometer_bias*0.95
        self.x[10:13] = self.magnetometer_bias *0.95
        # self.x[21] = self.pressure_bias
        # self.x[22] = self.temperature_bias
        
        self.R = np.diag([*self.accelerometer_std_deviation, *self.magnetometer_std_deviation])**2
        
        self.mag_declination = mag_declination
        
        print("Navigation EKF initialized")
        
    def filter_step(self, input_vector, measurement_vector, dt):    
        # predict
        print(input_vector)
        x_prediction, normalization_factor = self.state_transition_model_function(self.x, input_vector, dt)

        # compute process noise matrix
        L = self.state_transition_model_function_jacobian_inputs(self.x, dt, 1)
    
        quaternion_integration_process_noise_matrix = np.dot(L, np.dot(np.diag(self.gyroscope_std_deviation**2), L.transpose()))
        
        Q = np.eye(self.state_space_dim)*1e-6
        Q += quaternion_integration_process_noise_matrix
        
        F = self.state_transition_model_function_jacobian(self.x, input_vector, normalization_factor, dt)
        P_prediction = np.dot(F, np.dot(self.P, F.transpose())) + Q
        
        # update
        x_prediction[0:4] = x_prediction[0:4] / np.sqrt(np.sum(x_prediction[0:4]**2))
        H = self.prediction_measurement_vector_jacobian_gps(x_prediction)
        y = measurement_vector - self.prediction_measurement_vector_gps(x_prediction)
        # print(measurement_vector)
        # print(self.prediction_measurement_vector_gps(x_prediction))

        S = H.dot(P_prediction).dot(H.T) + self.R
        K = P_prediction.dot(H.T).dot(np.linalg.inv(S))
        
        self.x = x_prediction + K.dot(y)
        # self.x[7:] = x_prediction[7:]

        self.P = (np.eye(self.state_space_dim) - K.dot(H)).dot(P_prediction)
        
        # normalize quaternion
        self.x[0:4] = self.x[0:4] / np.sqrt(np.sum(self.x[0:4]**2))
        self.P = (self.P + self.P.T)/2.0
        
        return self.x
    

    def state_transition_model_function(self, x, u, dt):
        # enu frame
        q0 = x[0]
        q1 = x[1]
        q2 = x[2]
        q3 = x[3]
        
        # rocket frame
        wx = u[0]
        wy = u[1]
        wz = u[2]
        
        wbx = x[4]
        wby = x[5]
        wbz = x[6]
        
        x_new = x.copy()
            
        x_new[0] += (-q1*(wx-wbx) - q2*(wy-wby) - q3*(wz-wbz)) * dt/2
        x_new[1] += ( q0*(wx-wbx) - q3*(wy-wby) + q2*(wz-wbz)) * dt/2
        x_new[2] += ( q3*(wx-wbx) + q0*(wy-wby) - q1*(wz-wbz)) * dt/2
        x_new[3] += (-q2*(wx-wbx) + q1*(wy-wby) + q0*(wz-wbz)) * dt/2
        
        normalization_factor = 1 / np.sqrt(np.sum(x_new[0:4]**2))
        x_new[0:4] = x_new[0:4] * normalization_factor

        return x_new, normalization_factor
    
    def state_transition_model_function_jacobian(self, x, u, normalization_factor, dt):
        F = np.eye(self.state_space_dim)
        F[4:13, 4:13]=np.eye(9)*0.01
        
        q0 = x[0]
        q1 = x[1]
        q2 = x[2]
        q3 = x[3]
        
        wx = u[0]
        wy = u[1]
        wz = u[2]
        
        wbx = x[4]
        wby = x[5]
        wbz = x[6]     
                                    
        # q0 = (q0 + (-q1 * (wx - wbx) - q2 * (wy - wby) - q3 * (wz - wbz) * dt/2) * normalization_factor
        F[0][0] =  normalization_factor
        F[0][1] = -normalization_factor * (wx - wbx) * dt/2
        F[0][2] = -normalization_factor * (wy - wby) * dt/2
        F[0][3] = -normalization_factor * (wz - wbz) * dt/2
        
        F[0][4] =  normalization_factor * q1 * dt/2
        F[0][5] =  normalization_factor * q2 * dt/2
        F[0][6] =  normalization_factor * q3 * dt/2
        
        # q1 = (q1 + (q0 * (wx - wbx) - q3 * (wy - wby) + q2 * (wz - wbz)) * dt/2) * normalization_factor
        F[1][0] =  normalization_factor * (wx - wbx) * dt/2
        F[1][1] =  normalization_factor
        F[1][2] =  normalization_factor * (wz - wbz) * dt/2
        F[1][3] = -normalization_factor * (wy - wby) * dt/2
        
        F[1][4] = -normalization_factor * q0 * dt/2
        F[1][5] =  normalization_factor * q3 * dt/2
        F[1][6] = -normalization_factor * q2 * dt/2
        
        # q2 = (q2 + (q3 * (wx - wbx) + q0 * (wy - wby) - q1 * (wz - wbz)) * dt/2) * normalization_factor
        F[2][0] =  normalization_factor * (wy - wby) * dt/2
        F[2][1] = -normalization_factor * (wz - wbz) * dt/2
        F[2][2] =  normalization_factor
        F[2][3] =  normalization_factor * (wx - wbx) * dt/2
        
        F[2][4] = -normalization_factor * q3 * dt/2
        F[2][5] = -normalization_factor * q0 * dt/2
        F[2][6] =  normalization_factor * q1 * dt/2
    
        # q3 = (q3 + (-q2 * (wx - wbx) + q1 * (wy - wby) + q0 * (wz - wbz)) * dt/2) * normalization_factor
        F[3][0] =  normalization_factor * (wz - wbz) * dt/2
        F[3][1] =  normalization_factor * (wy - wby) * dt/2
        F[3][2] = -normalization_factor * (wx - wbx) * dt/2
        F[3][3] =  normalization_factor
        
        F[3][4] =  normalization_factor * q2 * dt/2
        F[3][5] = -normalization_factor * q1 * dt/2
        F[3][6] =  normalization_factor * q0 * dt/2
        
        return F

    def state_transition_model_function_jacobian_inputs(self, x, dt, normalization_factor):
        F = np.zeros((self.state_space_dim, 3), dtype=float)
        
        q0 = x[0]
        q1 = x[1]
        q2 = x[2]
        q3 = x[3]

        # q0 = (q0 + (-q1 * (wx - wbx) - q2 * (wy - wby) - q3 * (wz - wbz) * dt/2) * normalization_factor
        F[0][0] = -normalization_factor * q1 * dt/2
        F[0][1] = -normalization_factor * q2 * dt/2
        F[0][2] = -normalization_factor * q3 * dt/2
        
        # q1 = (q1 + (q0 * (wx - wbx) - q3 * (wy - wby) + q2 * (wz - wbz)) * dt/2) * normalization_factor
        F[1][0] =  normalization_factor * q0 * dt/2
        F[1][1] = -normalization_factor * q3 * dt/2
        F[1][2] =  normalization_factor * q2 * dt/2
        
        # q2 = (q2 + (q3 * (wx - wbx) + q0 * (wy - wby) - q1 * (wz - wbz)) * dt/2) * normalization_factor
        F[2][0] =  normalization_factor * q3 * dt/2
        F[2][1] =  normalization_factor * q0 * dt/2
        F[2][2] = -normalization_factor * q1 * dt/2
    
        # q3 = (q3 + (-q2 * (wx - wbx) + q1 * (wy - wby) + q0 * (wz - wbz)) * dt/2) * normalization_factor
        F[3][0] = -normalization_factor * q2 * dt/2
        F[3][1] =  normalization_factor * q1 * dt/2
        F[3][2] = -normalization_factor * q0 * dt/2
      
        return F


    def prediction_measurement_vector_gps(self, x):
        q0, q1, q2, q3 = x[0:4]

        M = quaternion(q0, q1, q2, q3).DCM().T # compute ENU to RF rotation matrix
        
        # magnetometer measurements (3 -> 5)
        mx_enu = np.sin(self.mag_declination)
        my_enu = np.cos(self.mag_declination)
        mz_enu = 0.0

        mag_enu = np.array([mx_enu, my_enu, mz_enu])        
        mag = np.dot(M, mag_enu)
        
        g = np.array([0, 0, 1])
        g_rf = np.dot(M, g)
        
        return np.array([*g_rf, *mag]) + np.array([*x[7:10], *x[10:13]])
    

    def prediction_measurement_vector_jacobian_gps(self, x):
        H = np.zeros((self.measurement_space_dim, self.state_space_dim))
                
        q0, q1, q2, q3 = x[0:4]
        
        # magnetometer measurements (3 -> 5)
        mx_enu = np.sin(self.mag_declination)
        my_enu = np.cos(self.mag_declination)
        mz_enu = 0.0
                
        # ax = 2 * g * (q1 * q3 + q2 * q0)
        H[0][0] = -2*q2
        H[0][1] = +2*q3
        H[0][2] = -2*q0
        H[0][3] = +2*q1
        
        # ay = 2 * g * (q2 * q3 - q1 * q0)
        H[1][0] = 2*q1
        H[1][1] = 2*q0
        H[1][2] = 2*q3
        H[1][3] = 2*q2
        
        # az = g * (q0**2 - q1**2 - q2**2 + q3**2)
        H[2][0] = +2*q0
        H[2][1] = -2*q1
        H[2][2] = -2*q2
        H[2][3] = +2*q3

        H[0:3,7:10] = np.eye(3)

        # (q0**2 + q1**2 - q2**2 - q3**2)*mx_enu + 2*(q1*q2+q0*q3)*my_enu + 2*(q1*q3-q0*q2)*mz_enu
        H[3][0] = 2*( q0*mx_enu + q3*my_enu - q2*mz_enu)
        H[3][1] = 2*( q1*mx_enu + q2*my_enu + q3*mz_enu)
        H[3][2] = 2*(-q2*mx_enu + q1*my_enu - q0*mz_enu)
        H[3][3] = 2*(-q3*mx_enu + q0*my_enu + q1*mz_enu)
        
        #my = 2*(q1*q2-q0*q3)*mx_enu + (q0**2 - q1**2 + q2**2 - q3**2)*my_enu + 2*(q2*q3+q0*q1)*mz_enu
        H[4][0] = 2*(-q3*mx_enu + q0*my_enu + q1*mz_enu)
        H[4][1] = 2*( q2*mx_enu - q1*my_enu + q0*mz_enu)
        H[4][2] = 2*( q1*mx_enu + q2*my_enu + q3*mz_enu)
        H[4][3] = 2*(-q0*mx_enu - q3*my_enu + q2*mz_enu)
        
        #mz = 2*(q1*q3+q0*q2)*mx_enu + 2*(q2*q3-q0*q1)*my_enu + (q0**2 - q1**2 - q2**2 + q3**2)*mz_enu
        H[5][0] = 2*( q2*mx_enu - q1*my_enu + q0*mz_enu)
        H[5][1] = 2*( q3*mx_enu - q0*my_enu - q1*mz_enu)
        H[5][2] = 2*( q0*mx_enu + q3*my_enu - q2*mz_enu)
        H[5][3] = 2*( q1*mx_enu + q2*my_enu + q3*mz_enu)

        H[3:6,10:13] = np.eye(3)
      
        return H
    