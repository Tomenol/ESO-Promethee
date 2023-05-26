# -*- coding: utf-8 -*-
"""
Created on Wed Jan 26 15:40:31 2022

@author: Thomas Maynadié
"""

import numpy as np
import matplotlib.pyplot as plt

class gyroscope_preprocessor():
    def __init__(self, gyroscope_std_deviation_1, gyroscope_bias_1, gyroscope_std_deviation_2, gyroscope_bias_2, f0=5):
        # fuses and filters 2 gyroscope measurements to estimate angular velocity and acceleration
        self.gyroscope_std_deviation_1 = gyroscope_std_deviation_1
        self.gyroscope_std_deviation_2 = gyroscope_std_deviation_2
        
        self.virtual_gyroscope_std_deviation = np.sqrt((gyroscope_std_deviation_1 * gyroscope_std_deviation_2) / (gyroscope_std_deviation_1**2 + gyroscope_std_deviation_2**2))
        self.virtual_gyroscope_bias = (gyroscope_bias_1 * gyroscope_std_deviation_2**2 + gyroscope_bias_2 * gyroscope_std_deviation_1**2) / (gyroscope_std_deviation_1**2 + gyroscope_std_deviation_2**2)
        
        self.gyroscope_prefilter = gyroscope_prefilter(self.virtual_gyroscope_std_deviation, self.virtual_gyroscope_bias)
        
        self.lpf = low_pass_filter(1, 1, f0*2*np.pi, np.zeros(3))
        
    def process_measurements(self, gyroscope_measurement_1, gyroscope_measurement_2, dt):
        fused_measurements = self.__fuse_measurements(gyroscope_measurement_1, gyroscope_measurement_2)
        
        return fused_measurements# self.gyroscope_prefilter.filter_measurements(fused_measurements, dt)
    
    def __fuse_measurements(self, gyroscope_measurement_1, gyroscope_measurement_2):
        mean_measurement = (gyroscope_measurement_1 * self.gyroscope_std_deviation_2**2 + gyroscope_measurement_2 * self.gyroscope_std_deviation_1**2) / (self.gyroscope_std_deviation_1**2 + self.gyroscope_std_deviation_2**2)

        return mean_measurement
    
    def get_std_deviation(self):
        return self.virtual_gyroscope_std_deviation
    
    def get_bias(self):
        return self.virtual_gyroscope_bias

class low_pass_filter:
    def __init__(self, K, xhi, w0, y0):
        self.y_pp = y0
        self.y_p = y0
        
        self.xhi = xhi
        self.w0 = w0
        self.K = K
        
    def out(self, x, dt):
        y = (2*(1 + self.xhi*self.w0*dt)*self.y_p - self.y_pp + self.K*dt**2*self.w0**2*x)/((self.w0*dt)**2 + 2*self.xhi*self.w0*dt + 1)
        
        self.y_pp = self.y_p
        self.y_p = y
        
        return y
        
    
class gyroscope_prefilter():
    def __init__(self, gyroscope_std_deviation, gyroscope_bias):
        # state vector : x = [wx, wy, wz, dwx, dwy, dwz] -> 9
        # measurement vector : x = [wx, wy, wz] -> 3
        
        # spaces dimensions
        self.ndim = 6
        self.nmeas = 3 
        
        # state and KF variables
        self.x = np.zeros(self.ndim)
        self.P = np.eye(self.ndim)*1e-3
        
        self.R = np.diag(gyroscope_std_deviation**2)
        self.std_deviation = gyroscope_std_deviation
        self.bias = gyroscope_bias
                
        # prediction model
        self.F = np.eye(self.ndim)
        
        # measurement model
        self.H = np.zeros((self.nmeas, self.ndim))
        self.H[0:3,0:3] = np.eye(self.nmeas)
        
    def filter_measurements(self, measurements, dt):
        # prediction model
        self.F[0:3,3:6] = np.eye(3)*dt
        self.Q = np.diag([0, 0, 0, *self.std_deviation**2 * dt/2])
        
        # prediction
        x_prediction = np.dot(self.F, self.x)
        P_prediction = np.dot(self.F, np.dot(self.P, self.F.transpose())) + self.Q
        
        # update        
        y = measurements - self.bias*0 - np.dot(self.H, x_prediction)
        S = np.dot(self.H, np.dot(P_prediction, self.H.transpose())) + self.R
        
        K = np.dot(P_prediction, np.dot(self.H.transpose(), np.linalg.inv(S)))
        
        self.x = x_prediction + np.dot(K, y)
        self.P = np.dot((np.eye(self.ndim) - np.dot(K, self.H)), P_prediction)
        
        self.P = (self.P + self.P.transpose())/2
             
        return measurements, self.x[3:6]     
    
class imu_sensor_preprocessor():
    def __init__(self, sensor_std_deviation_1, sensor_bias_1, sensor_std_deviation_2, sensor_bias_2):
        # fuses 2 imu measurements
        self.sensor_std_deviation_1 = sensor_std_deviation_1
        self.sensor_std_deviation_2 = sensor_std_deviation_2
        
        self.sensor_bias_1 = sensor_bias_1
        self.sensor_bias_2 = sensor_bias_2
        
        self.virtual_sensor_std_deviation = np.sqrt((self.sensor_std_deviation_1 * self.sensor_std_deviation_2) / (self.sensor_std_deviation_1**2 + self.sensor_std_deviation_2**2))
        self.virtual_sensor_bias = (self.sensor_bias_1 * self.sensor_std_deviation_2**2 + self.sensor_bias_2 * self.sensor_std_deviation_1**2) / (self.sensor_std_deviation_1**2 + self.sensor_std_deviation_2**2)
      
    def process_measurements(self, measurement_1, measurement_2):
        return (measurement_1 * self.sensor_std_deviation_2**2 + measurement_2 * self.sensor_std_deviation_1**2) / (self.sensor_std_deviation_1**2 + self.sensor_std_deviation_2**2)
        
    def get_std_deviation(self):
        return self.virtual_sensor_std_deviation
    
    def get_bias(self):
        return self.virtual_sensor_bias