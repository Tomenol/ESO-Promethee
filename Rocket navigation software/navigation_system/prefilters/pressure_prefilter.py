# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 20:34:08 2022

@author: Thomas Maynadié
"""

import numpy as np
import matplotlib.pyplot as plt
from .atmospheric_model import ISA

from quaternion_helper import quaternion

class Pressure_prefilter:
    def __init__(self, pressure_std_deviation, pressure_bias, gps_std_deviation, accelerometer_std_deviation, accelerometer_bias, z_0, P_0, initial_attitude):
        self.pressure_std_deviation = pressure_std_deviation
        self.pressure_bias = pressure_bias
        
        self.accelerometer_std_deviation = accelerometer_std_deviation
        self.accelerometer_bias = accelerometer_bias
        
        self.gps_z_std_deviation = gps_std_deviation[2]
        
        self.initial_attitude = initial_attitude
        
        # x = [z, vz, az, b_P_isa].T
        # x : state vector / P : covariance matrix
        self.state_space_dimension = 4
        
        self.state_vector = np.array([z_0, 0, 0, self.pressure_bias])
        self.P = np.eye(self.state_space_dimension) * 1
        
        # ISA :
        self.atmospheric_model = ISA()
   
        self.z0 = z_0
        
        P_init_isa, T_init_isa = self.atmospheric_model.get_predictions(self.z0)
        
        # initial temperature and pressure biases
        self.b_P_isa = P_0 - P_init_isa
        
    def output(self, input_vector, measurement_vector, dt, accelerometer_bias, accelerometer_std_deviation, use_accel=False, use_gps=False):
        Q = np.array([*(np.ones(3)*dt*0.00001), self.pressure_std_deviation])**2
        
        # setup measurement vector
        measurement_vector, R = self.__compute_measurement_vector(measurement_vector, accelerometer_bias, accelerometer_std_deviation, use_accel, use_gps)
        
        # predict
        F = self.__state_evolution_matrix(self.state_vector, input_vector, dt)
        x_prediction = np.dot(F, self.state_vector)
        P_prediction = np.dot(F, np.dot(self.P, F.transpose())) + Q
        
        # update
        measurement_model = self.__measurement_model(use_accel, use_gps)
        
        predicted_measurement = measurement_model(self.state_vector)
        H = self.__get_jacobian_matrix(measurement_model, len(predicted_measurement), self.state_vector, 0.01, args=())
        
        y = measurement_vector - predicted_measurement
        
        S = np.dot(H, np.dot(P_prediction, H.transpose())) + R
        K = np.dot(P_prediction, np.dot(H.transpose(), np.linalg.inv(S)))
        
        self.state_vector = x_prediction + np.dot(K, y)
                
        self.P = np.dot((np.eye(self.state_space_dimension) - np.dot(K, H)), P_prediction)
        self.P = (self.P + self.P.transpose())/2
        
        return self.state_vector
    
    def __state_evolution_matrix(self, state_vector, input_vector, dt):        
        F = np.eye(self.state_space_dimension)
        
        F[0][1] = dt
        F[1][2] = dt
        
        return F
        
    def __measurement_model(self, use_accel, use_gps):    
        if use_accel and use_gps:           measurement_model = self.__measurement_model_accel_gps
        elif not use_accel and use_gps:     measurement_model = self.__measurement_model_no_accel_gps
        elif use_accel and not use_gps:     measurement_model = self.__measurement_model_accel_no_gps
        else:                               measurement_model = self.__measurement_model_no_accel_no_gps
        
        return measurement_model
            
    def __measurement_model_accel_gps(self, state_vector):
        return np.array([state_vector[0], state_vector[2], self.__compute_pressure_measurement(state_vector)]) # z = [z, az, P]
    
    def __measurement_model_no_accel_gps(self, state_vector):
        return np.array([state_vector[0], self.__compute_pressure_measurement(state_vector)]) # z = [z, P]
    
    def __measurement_model_accel_no_gps(self, state_vector):
        return np.array([state_vector[2], self.__compute_pressure_measurement(state_vector)]) # z = [az, P]
    
    def __measurement_model_no_accel_no_gps(self, state_vector):
        return np.array([self.__compute_pressure_measurement(state_vector)]) # z = [P]
    
    def __compute_measurement_vector(self, measurement_vector, accelerometer_bias, accelerometer_std_deviation, use_accel, use_gps):
        # measurement_vector_raw = [x, P, ax, ay, az]
        
        new_measurement_vector = []
        measurement_noise = []
        
        if use_gps: 
            new_measurement_vector.append(measurement_vector[0])
            measurement_noise.append(self.gps_z_std_deviation**2)
        
        if use_accel:  
            rotation_matrix = quaternion(*self.initial_attitude).DCM()
            
            acceleration_measurement = np.dot(rotation_matrix, measurement_vector[2:5])
            
            accelerometer_bias = np.dot(rotation_matrix, accelerometer_bias)
            accelerometer_std_deviation = np.dot(rotation_matrix, accelerometer_std_deviation)
            
            new_measurement_vector.append(acceleration_measurement[2] - accelerometer_bias[2])
            measurement_noise.append((self.gps_z_std_deviation)**2)
            
        new_measurement_vector.append(measurement_vector[1])
        measurement_noise.append((self.pressure_std_deviation)**2)

        return np.array(new_measurement_vector), np.diag(measurement_noise)
    
    def __get_jacobian_matrix(self, function, function_size, x, dalpha, args=()):
        n = len(x)
        
        J = np.zeros((function_size, n))
        
        for i in range(function_size):
            for j in range(n):
                dx = np.zeros(n)
                dx[j] = dalpha
                
                J[i][j] = (function(x + dx, *args)[i] - function(x - dx, *args)[i])/(2*dalpha)
                        
        return J
    
    def __compute_pressure_measurement(self, state_vector):
        return self.b_P_isa + self.atmospheric_model.get_predictions(state_vector[0])[0]
    
    
class Altitude_simulation:
    def __init__(self, pressure_std_deviation, pressure_bias, gps_std_deviation, z_0, P_0, initial_attitude, az, tf, n):
        self.pressure_std_deviation = pressure_std_deviation
        self.pressure_bias = pressure_bias
        self.gps_std_deviation = gps_std_deviation

        self.initial_attitude = initial_attitude
        self.az = az
        
        self.isa = ISA()
        self.z_0 = z_0
        self.P_0 = P_0
                
        P_init_isa, T_init_isa = self.isa.get_predictions(self.z_0)
        
        # initial temperature and pressure biases
        self.b_P_isa = P_0 - P_init_isa
        
        self.t = np.linspace(0, tf, n)
        
        self.x = np.zeros([4, n]) # z, vz, az, bP
        self.P_isa = np.ones(n) * 101325
        
        self.x[0][0] = z_0
        self.x[2][0] = self.az
        self.x[3][0] = self.b_P_isa + self.pressure_bias
        
        for i in range(1, n):
            dt = self.t[i] - self.t[i-1]
            
            self.x[0][i] = self.x[0][i-1] + self.x[1][i-1] * dt      # z
            self.x[1][i] = self.x[1][i-1] + self.x[2][i-1] * dt      # vz
            self.x[2][i] = self.az                  # az
            self.x[3][i] = self.x[3][i-1]         # bP

            self.P_isa[i] = self.isa.get_predictions(self.x[0][i])[0]
                    
    def get_measurements(self, accelerometer_bias, accelerometer_std_deviation, k):
        rotation_matrix_inv = np.transpose(quaternion(*self.initial_attitude).DCM())
        
        acceleration_measurement = np.dot(rotation_matrix_inv, [0, 0, self.x[2][k]]) + np.random.normal(accelerometer_bias, accelerometer_std_deviation, 3)
        
        pressure_measurement = self.P_isa[k] + self.x[3][k-1] + np.random.normal(0, self.pressure_std_deviation, 1)
        gps_measurement = self.x[0][k] + np.random.normal(0, self.gps_std_deviation[2], 1)
        
        return np.array([gps_measurement, pressure_measurement, *accelerometer_std_deviation])
    
    def get_state_vector(self):
        return self.x
        
        
        
        
        
        
        
        
        
        
        