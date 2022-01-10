# -*- coding: utf-8 -*-
"""
Created on Sat Dec 18 15:43:23 2021

Kalman filter 

@author: Thomas Maynadié
"""

# imports 
import numpy as np
import time
import serial

# sensor parameters

# kalman

def getNextValidData():
    tmp_dat = 0
   
    while True:
        iflag = 0
        
        try: tmp_dat = arduino.readline().split()[0].decode("utf-8")
        except IndexError: 
            iflag = 1
            print("Error : no valid data found")
            
        if iflag == 0: break 
            
    return tmp_dat
        
def read_serial_buffer():
    data = []
    tmp_dat = 0
        
    while True:
        tmp_dat = getNextValidData()
        
        if (tmp_dat != str(0xEEFE)): print("awaiting serial data")
        else: break

    while  True:
        tmp_dat = getNextValidData()
        
        if (tmp_dat != str(0xEEFD)):
            data.append(float(tmp_dat))
        else: break
        
    return data

def get_measurements():
    data = read_serial_buffer()
        
    gps = (np.random.rand(6) - 0.5)*0.01
    
    accel1 = np.array([data[0], data[1], data[2]])
    accel2 = np.array([data[0], data[1], data[2]])
    
    gyro1 = np.array([data[3], data[4], data[5]])
    gyro2 = np.array([data[3], data[4], data[5]])
    
    mag1 = np.array([data[6], data[7], data[8]])
    mag2 = np.array([data[6], data[7], data[8]])
    
    return gps, accel1, accel2, gyro1, gyro2, mag1, mag2

def rotation_matrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta[0]), -np.sin(theta[0])],
                    [0, np.sin(theta[0]), np.cos(theta[0])]])
    
    R_y = np.array([[np.cos(theta[1]), 0, np.sin(theta[1])],
                    [0, 1, 0],
                    [-np.sin(theta[1]), 0, np.cos(theta[1])]])
    
    R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                    [np.sin(theta[2]), np.cos(theta[2]), 0],
                    [0, 0, 1]])

    return np.array(np.dot(R_z, np.dot(R_y, R_x)))

def kalman_gps(X, orientation, velocity, gps, mag, g, dt):   
    global K, Q, R, P
    
    F = state_transition_model_function_jacobian(X, orientation, velocity, g, dt)
    
    # predict
    x_prediction = state_transition_model_function(X, orientation, velocity, g, dt)
    P_prediction = np.dot(F, np.dot(P, F.transpose())) + Q
    
    # update
    z = measurement_vector_gps(gps, mag, orientation)
    H = prediction_measurement_vector_jacobian_gps(x_prediction)
    
    y = z - prediction_measurement_vector_gps(x_prediction)
    S = np.dot(H, np.dot(P_prediction, H.transpose())) + R
    
    K = np.dot(P_prediction, np.dot(H.transpose(), np.linalg.inv(S)))
    
    X = x_prediction + np.dot(K, y)
    P = np.dot((np.eye(22) - np.dot(K, H)), P_prediction)

    return X

def prediction_measurement_vector_jacobian_gps(x):
    H = np.zeros([12, 22])   
    
    # phi
    v = x[0]**2 - x[1]**2 - x[2]**2 + x[3]**2

    if (v != 0):
        u = x[2] * x[3] + x[0] * x[1]
        v2 = v**2
        beta = (2*u/v)**2
        
        H[0][0] = 2 * (x[1]*v - 2*x[0]*u)/(v2 * (1 + beta))
        H[0][1] = 2 * (x[0]*v + 2*x[1]*u)/(v2 * (1 + beta))
        H[0][2] = 2 * (x[3]*v + 2*x[2]*u)/(v2 * (1 + beta))
        H[0][3] = 2 * (x[2]*v - 2*x[3]*u)/(v2 * (1 + beta))
    
    # theta 
    u = 2*(x[1]*x[3] - x[0]*x[2])
    
    if (u != 0):
        H[1][0] =  2 * x[2] / np.sqrt(1 - u**2)
        H[1][1] = -2 * x[3] / np.sqrt(1 - u**2)
        H[1][2] =  2 * x[0] / np.sqrt(1 - u**2)
        H[1][3] = -2 * x[1] / np.sqrt(1 - u**2)
        
    # psi  
    v = x[0]**2 + x[1]**2 - x[2]**2 - x[3]**2

    if (v != 0):
        v2 = v**2
        u = 2 * (x[1] * x[2] + x[0] * x[3])
        
        beta = (u/v)**2
        
        H[2][0] = 2 * (x[3] * v  - x[0] * u) / (v2 * (1 + beta))
        H[2][1] = 2 * (x[2] * v  - x[1] * u) / (v2 * (1 + beta))
        H[2][2] = 2 * (x[1] * v  + x[2] * u) / (v2 * (1 + beta))
        H[2][3] = 2 * (x[0] * v  + x[3] * u) / (v2 * (1 + beta))
    
    H[3][4] = 1
    H[4][5] = 1
    H[5][6] = 1
    H[6][7] = 1
    H[7][8] = 1
    H[8][9] = 1

    H[9][16] = 1
    H[10][17] = 1
    H[11][18] = 1
    
    return H

def get_measurement_properties(n_measurements):
    measurements = []
    measurements_imu = []
    
    for i in range(n_measurements):    
        gps, accel1, accel2, gyro1, gyro2, mag1, mag2 = get_measurements()
        
        accel = 0.5 * (accel1 + accel2)
        gyro = 0.5 * (gyro1 + gyro2)
        mag = 0.5 * (mag1 + mag2)
        
        measurements.append([*gps, *mag])
        measurements_imu.append([*accel, *gyro])
    
    return np.mean(measurements, axis=0), np.var(measurements, axis=0), np.mean(measurements_imu, axis=0), np.var(measurements_imu, axis=0)

def prediction_measurement_vector_gps(x):
    phi = np.arctan2(2*(x[2] * x[3] + x[0] * x[1]), x[0]**2 - x[1]**2 - x[2]**2 + x[3]**2)
    theta = -np.arcsin(2*(x[1] * x[3] - x[0] * x[2]))
    psi = np.arctan2(2*(x[1] * x[2] + x[0] * x[3]), x[0]**2 + x[1]**2 - x[2]**2 - x[3]**2)
    
    return np.array([phi, theta, psi, x[4], x[5], x[6], x[7], x[8], x[9], x[16], x[17], x[18]]) #[gps_pos, gps_vel, mag, angles]
    

def measurement_vector_gps(gps, mag, orientation):
    return np.array([
        orientation[0],
        orientation[1],
        orientation[2],
        gps[0],
        gps[1],
        gps[2],
        gps[3],
        gps[4],
        gps[5],
        mag[0],
        mag[1],
        mag[2]
        ])

def state_transition_model_function(x, ang, vel, g, dt):
    return np.array(
        [x[0] - x[1] * (ang[0] - x[10]) * 0.5 - x[2] * (ang[1] - x[11]) * 0.5 - x[3] * (ang[2] - x[12]) * 0.5,
         x[1] - x[0] * (ang[0] - x[10]) * 0.5 - x[3] * (ang[1] - x[11]) * 0.5 + x[2] * (ang[2] - x[12]) * 0.5,
         x[2] + x[3] * (ang[0] - x[10]) * 0.5 + x[0] * (ang[1] - x[11]) * 0.5 - x[1] * (ang[2] - x[12]) * 0.5,
         x[3] - x[2] * (ang[0] - x[10]) * 0.5 + x[1] * (ang[1] - x[11]) * 0.5 + x[0] * (ang[2] - x[12]) * 0.5,
         x[4] + dt * x[7],
         x[5] + dt * x[8],
         x[6] + dt * x[9],
         x[7] + (vel[0] - x[13]) * (x[0]**2 + x[1]**2 - x[2]**2 - x[3]**2) - 2*(vel[1] - x[14]) *(x[0]*x[3] - x[1]*x[2]) + 2*(vel[2] - x[15]) * (x[0]*x[2] + x[1]*x[3]),
         x[8] + (vel[1] - x[14]) * (x[0]**2 - x[1]**2 + x[2]**2 - x[3]**2) + 2*(vel[0] - x[13]) *(x[0]*x[3] + x[1]*x[2]) - 2*(vel[2] - x[15]) * (x[0]*x[1] - x[2]*x[3]),
         x[9] + (vel[2] - x[15]) * (x[0]**2 - x[1]**2 - x[2]**2 + x[3]**2) - 2*(vel[0] - x[13]) *(x[0]*x[2] - x[1]*x[3]) + 2*(vel[1] - x[14]) * (x[0]*x[1] + x[2]*x[3]),
         x[10],
         x[11],
         x[12],
         x[13],
         x[14],
         x[15],
         x[16],
         x[17],
         x[18],
         x[19],
         x[20],
         x[21]])
         
def state_transition_model_function_jacobian(x, ang, vel, g, dt):
    F = np.eye(22)
    
    # x[0]
    F[0][1] = 0.5 * (x[10] - ang[0])
    F[0][2] = 0.5 * (x[11] - ang[1])
    F[0][3] = 0.5 * (x[12] - ang[2])
    F[0][10] = 0.5 * x[1]
    F[0][11] = 0.5 * x[2]
    F[0][12] = 0.5 * x[3]
    
    # x[1]
    F[1][0] = 0.5 * (x[10] - ang[0])
    F[1][2] = 0.5 * (ang[2] - x[12])
    F[1][3] = 0.5 * (x[11] - ang[1])
    F[1][10] = 0.5 * x[0]
    F[1][11] = 0.5 * x[3]
    F[1][12] = -0.5 * x[2]
    
    # x[2]
    F[2][0] = 0.5 * (ang[1] - x[11])
    F[2][1] = 0.5 * (x[12] - ang[2])
    F[2][3] = 0.5 * (ang[0] - x[10])
    F[2][10] = -0.5 * x[3]
    F[2][11] = -0.5 * x[0]
    F[2][12] = 0.5 * x[1]
    
    # x[3]
    F[3][0] = 0.5 * (ang[2] - x[12])
    F[3][1] = 0.5 * (ang[1] - x[11])
    F[3][2] = 0.5 * (x[10] - ang[0])
    F[3][10] = 0.5 * x[2]
    F[3][11] = -0.5 * x[1]
    F[3][12] = -0.5 * x[0]
    
    # x[4]
    F[4][7] = dt
 
    # x[5]
    F[5][8] = dt
 
    # x[6]
    F[6][9] = dt
    
    # x[7]
    F[7][0] = 2 * (x[0] * (vel[0] - x[13]) + x[3] * (x[14] - vel[1]) + x[2] * (vel[2] - x[15]))
    F[7][1] = 2 * (x[1] * (vel[0] - x[13]) + x[2] * (vel[1] - x[14]) + x[3] * (vel[2] - x[15]))
    F[7][2] = 2 * (x[2] * (x[13] - vel[0]) + x[1] * (vel[1] - x[14]) + x[0] * (vel[2] - x[15]))
    F[7][3] = 2 * (x[3] * (x[13] - vel[0]) + x[0] * (x[14] - vel[1]) + x[1] * (vel[2] - x[15]))
    F[7][13] = 2 * (x[2]**2 + x[3]**2 - x[0]**2 - x[1]**2)
    F[7][14] = 2 * (x[0] * x[3] - x[1] * x[2])
    F[7][15] = -2 * (x[0] * x[2] + x[1] * x[3])
    
    # x[8]
    F[8][0] = 2 * (x[0] * (vel[1] - x[14]) + x[3] * (vel[0] - x[13]) + x[1] * (x[15] - vel[2]))
    F[8][1] = 2 * (x[1] * (x[14] - vel[1]) + x[2] * (vel[0] - x[13]) + x[0] * (x[15] - vel[2]))
    F[8][2] = 2 * (x[2] * (vel[1] - x[14]) + x[1] * (vel[0] - x[13]) + x[3] * (vel[2] - x[15]))
    F[8][3] = 2 * (x[3] * (x[14] - vel[1]) + x[0] * (vel[0] - x[13]) + x[2] * (vel[2] - x[15]))
    F[8][13] = 2 * (x[1]**2 + x[3]**2 - x[0]**2 - x[2]**2)
    F[8][14] = -2 * (x[0] * x[3] + x[1] * x[2])
    F[8][15] = 2 * (x[2] * x[3] + x[0] * x[1])

    # x[9]
    F[9][0] = 2 * (x[0] * (vel[2] - x[15]) + x[2] * (x[13] - vel[0]) + x[1] * (vel[1] - x[14]))
    F[9][1] = 2 * (x[1] * (x[15] - vel[2]) + x[3] * (vel[0] - x[13]) + x[0] * (vel[1] - x[14]))
    F[9][2] = 2 * (x[2] * (x[15] - vel[2]) + x[0] * (x[13] - vel[0]) + x[3] * (vel[1] - x[14]))
    F[9][3] = 2 * (x[3] * (vel[2] - x[15]) + x[1] * (vel[0] - x[13]) + x[2] * (vel[1] - x[14]))
    F[9][13] = 2 * (x[1]**2 + x[2]**2 - x[0]**2 - x[3]**2)
    F[9][14] = 2 * (x[1] * x[3] - x[0] * x[2])
    F[9][15] = 2 * (x[0] * x[1] + x[2] * x[3])
    
    return F

def main(arduino):
    # Kalman variables
    # q0                            x[0]
    # q1                            x[1]
    # q2                            x[2]
    # q3                            x[3]
    # positionN                     x[4]
    # positionE                     x[5]
    # positionD                     x[6]
    # νN                            x[7]
    # νE                            x[8]
    # νD                            x[9]
    # ΔθbiasX                       x[10]
    # ΔθbiasY                       x[11]
    # ΔθbiasZ                       x[12]
    # ΔνbiasX                       x[13]
    # ΔνbiasY                       x[14]
    # ΔνbiasZ                       x[15]
    # geomagneticFieldVectorN       x[16]
    # geomagneticFieldVectorE       x[17]
    # geomagneticFieldVectorD       x[18]
    # magbiasX                      x[19]
    # magbiasY                      x[20]
    # magbiasZ                     x[21]
    
    global K, B, Q, R, P
    
    # comptute measurement noise
    measurements_mean, measurement_var, imu_measurement_mean, imu_measurement_var = get_measurement_properties(30)
    
    Q = np.zeros(22)
    R = np.eye(12) * 0.01
    
    state_vector = np.ones(22)*0.0001
    P = np.zeros([22, 22])
        
    last_angular_velocity_measurement = np.zeros(3)
    last_velocity_vector = np.zeros(3)
    last_acceleration_measurement = np.zeros(3)
    
    velocity_vector = np.zeros(3)
    orientation_vector = np.zeros(3)
    
    estimated_angular_angular_acceleration = np.zeros(3)
    
    gravity = np.array([0, 0, 1]) * 9.81

    # IMU relative position to center of mass of the rocket
    r_imu_1 = np.array([1, 0, 3])
    r_imu_2 = np.array([2, 0, 3])
    
    rotation_matrix_1 = rotation_matrix([0, 0, 0])
    rotation_matrix_2 = rotation_matrix([0, 0, 0])
    inv_rotation_matrix_1 = np.linalg.inv(rotation_matrix_1)
    inv_rotation_matrix_2 = np.linalg.inv(rotation_matrix_2)

    iflag = True
    
    logfile = open("DATA/log_experimental_data_{0}{1}{2}_{3}_{4}_{5}.fusex_dat".format(
    time.localtime().tm_mday, 
    time.localtime().tm_mon, 
    time.localtime().tm_year, 
    time.localtime().tm_hour, 
    time.localtime().tm_min, 
    time.localtime().tm_sec), "w+")
    
    print("{0:25}{1:15}{2:15}{3:15}{4:15}{5:15}{6:15}{7:15}{8:15}{9:15}{10:15}{11:15}{12:15}{13:15}{14:15}{15:15}{16:15}{17:15}{18:15}{19:15}{20:15}{21:15}{22:15}{23:15}{24:15}".format("time", "ax1", "ax2", "ay1", "ay2", "az1", "az2", "wx1", "wx2", "wy1", "wy2", "wz1", "wz2", "mx1", "mx2", "my1", "my2", "mz1", "mz2", "gps_x", "gps_y", "gps_z", "gps_vx", "gps_vy", "gps_vz"), file=logfile, flush=True)    
    
    print("starting loop")
    
    # time informations
    t_start = time.time()
    
    for i in range(1000):
        # get measurements
        gps, accel1, accel2, gyro1, gyro2, mag1, mag2 = get_measurements()
  
        # get time interval between measurements
        t_end = time.time()
        dt = t_end - t_start
        t_start = time.time()
                
        # print raw measurements to file
        print("{0:15.5f}{1:15.5f}{2:15.5f}{3:15.5f}{4:15.5f}{5:15.5f}{6:15.5f}{7:15.5f}{8:15.5f}{9:15.5f}{10:15.5f}{11:15.5f}{12:15.5f}{13:15.5f}{14:15.5f}{15:15.5f}{16:15.5f}{17:15.5f}{18:15.5f}{19:15.5f}{20:15.5f}{21:15.5f}{22:15.5f}{23:15.5f}{24:15.5f}"
              .format(t_end,
                      accel1[0], 
                      accel2[0], 
                      accel1[1], 
                      accel2[1], 
                      accel1[2], 
                      accel2[2], 
                      gyro1[0], 
                      gyro2[0], 
                      gyro1[1], 
                      gyro2[1], 
                      gyro1[2], 
                      gyro2[2], 
                      mag1[0], 
                      mag2[0], 
                      mag1[1], 
                      mag2[1], 
                      mag1[2], 
                      mag2[2],
                      gps[0],
                      gps[1],
                      gps[2],
                      gps[3],
                      gps[4],
                      gps[5],
                      ), file=logfile, flush=True)    
        
        # move imu measurements to center of mass    
        gyro1 = np.dot(gyro1, inv_rotation_matrix_1)*np.pi/180.0
        gyro2 = np.dot(gyro2, inv_rotation_matrix_2)*np.pi/180.0
        
        mag1 = np.dot(mag1, inv_rotation_matrix_1)
        mag2 = np.dot(mag2, inv_rotation_matrix_2)
        
        accel1 = np.dot(accel1, inv_rotation_matrix_1)*9.81
        accel2 = np.dot(accel2, inv_rotation_matrix_2)*9.81

        angular_velocity_measurement = 0.5 * (gyro1 + gyro2)
            
        accel1 = accel1 - np.dot(angular_velocity_measurement, np.dot(angular_velocity_measurement, r_imu_1)) - np.dot(estimated_angular_angular_acceleration, r_imu_1)
        accel2 = accel2 - np.dot(angular_velocity_measurement, np.dot(angular_velocity_measurement, r_imu_2)) - np.dot(estimated_angular_angular_acceleration, r_imu_2)
        acceleration_measurement = 0.5 * (accel1 + accel2)
        magnetometer_measurements = 0.5 * (mag1 + mag2)
        
        # integrate measurements
        velocity_vector += dt/2 * (acceleration_measurement + last_acceleration_measurement)
        position_vector += dt/2 * (velocity_vector + last_velocity_vector)
        orientation_vector += dt/2 * (angular_velocity_measurement + last_angular_velocity_measurement)
        
        # transform GPS measurements to NEU frame
        
        # exec kalman filter on measurements depending on measurements
        state_vector = kalman_gps(state_vector, orientation_vector, velocity_vector, gps, magnetometer_measurements, gravity, dt)
        print(prediction_measurement_vector_gps(state_vector)[0]*180/np.pi)
        
        # compute angular acceleration
        if dt != 0: estimated_angular_angular_acceleration = (angular_velocity_measurement - last_angular_velocity_measurement)/dt
        else: estimated_angular_angular_acceleration = np.zeros(3)
        
        last_angular_velocity_measurement = angular_velocity_measurement
        last_acceleration_measurement = acceleration_measurement
        last_velocity_vector = velocity_vector
    
    return 1

if __name__ == "__main__":
    # setup serial communication with arduino
    try:
        arduino = serial.Serial(port='COM3', baudrate=115200)
        iflag = True
        
    except(serial.SerialException):
        print("Error : could not connect to the arduino on COM3. Exiting...")
        iflag = False
                
    if iflag:
        arduino.rtscts = True
        arduino.dsrdtr = True
        
        arduino.flushInput()
        
        print("Kalman INS filter script")
        
        main(arduino)
        
        print("Terminating...")
        
        arduino.close()

