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

import matplotlib.pyplot as plt

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
        
    gps = (np.random.rand(6) - 0.5)*0.001
    
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

def kalman_gps(X, angular_vel, accel, gps, mag, dt):   
    global K, Q, R, P

    F = state_transition_model_function_jacobian(X, dt)
    
    # predict
    x_prediction = state_transition_model_function(X, dt)
    P_prediction = np.dot(F, np.dot(P, F.transpose())) + Q
    
    # update
    z = measurement_vector_gps(angular_vel, accel, gps, mag)
    H = prediction_measurement_vector_jacobian_gps(x_prediction)
    
    y = z - prediction_measurement_vector_gps(x_prediction)
    S = np.dot(H, np.dot(P_prediction, H.transpose())) + R
    
    K = np.dot(P_prediction, np.dot(H.transpose(), np.linalg.inv(S)))
    
    X = x_prediction + np.dot(K, y)
    P = np.dot((np.eye(n_dim) - np.dot(K, H)), P_prediction)

    return X

def prediction_measurement_vector_gps(x):
    return np.array([x[9] + x[18], # rocket frame
                    x[10] + x[19], # rocket frame
                    x[11] + x[20], # rocket frame
                    x[12] + x[21], # rocket frame
                    x[13] + x[22], # rocket frame
                    x[14] + x[23], # rocket frame
                    x[0], 
                    x[1], 
                    x[2], 
                    x[5], 
                    x[6], 
                    x[7], 
                    x[15] + x[24],
                    x[16] + x[25], 
                    x[17] + x[26]])

def prediction_measurement_vector_jacobian_gps(x):
    H = np.zeros([15, n_dim])   
    
    H[0][9]     = 1
    H[0][18]    = 1
    H[1][10]    = 1
    H[1][19]    = 1
    H[2][11]    = 1
    H[2][20]    = 1
    
    H[3][12]    = 1
    H[3][21]    = 1
    
    H[4][13]    = 1
    H[4][22]    = 1
    
    H[5][14]    = 1
    H[5][23]    = 1
    
    H[6][0]     = 1
    H[7][1]     = 1
    H[8][2]     = 1
    H[9][5]     = 1
    H[10][6]    = 1
    H[11][7]    = 1
    H[12][15]   = 1
    H[13][24]   = 1
    H[13][16]   = 1
    H[13][25]   = 1
    H[14][17]   = 1
    H[14][26]   = 1
    
    return H

def get_measurement_properties(n_measurements):
    meas_w = []
    meas_a = []
    meas_pos = []
    meas_vel = []
    meas_mag = []
    
    for i in range(n_measurements):    
        gps, accel1, accel2, gyro1, gyro2, mag1, mag2 = get_measurements()
        
        accel = 0.5 * (accel1 + accel2)
        gyro = 0.5 * (gyro1 + gyro2)
        mag = 0.5 * (mag1 + mag2)
        
        meas_w.append(gyro)
        meas_a.append(accel)
        meas_pos.append(gps[0:3])
        meas_vel.append(gps[3:6])
        meas_mag.append(mag)
    
    m_w = np.mean(meas_w, axis=0)
    m_a = np.mean(meas_a, axis=0)
    m_mag = np.mean(meas_mag, axis=0)
    
    s_w = np.var(meas_w, axis=0)
    s_a = np.var(meas_a, axis=0)
    s_pos = np.var(meas_pos, axis=0)
    s_vel = np.var(meas_vel, axis=0)
    s_mag = np.var(meas_mag, axis=0)
    
    return s_w, s_a, s_pos, s_vel, s_mag, m_w, m_a, m_mag

def measurement_vector_gps(angular_vel, accel, gps, mag):
    return np.array([
        *angular_vel, # rocket frame
        *accel, # rocket frame
        *gps, # ENU
        *mag # rocket frame
        ])

def get_rocket_rotation_matrix(roll, pitch, yaw):
    sr = np.sin(roll)
    sp = np.sin(pitch)
    sy = np.sin(yaw)
    
    cr = np.cos(roll)
    cp = np.cos(pitch)
    cy = np.cos(yaw)
    
    M_rot = np.array([[cy*cp,       sr*sp*cy - sy*cr,       sr*sy+sp*cy*cr],
                      [cp*sy,       sr*sp*sy + cy*cr,      -sr*cy+sp*sy*cr],
                      [-sp,         sr*cp,                  cp*cr]])
    
    
    return M_rot

def state_transition_model_function(x, dt):
    x_new = x.copy()
    
    M_rot = get_rocket_rotation_matrix(*x[3:6])
    
    x_new[0:3] += np.dot(M_rot, (x[6:9] + x[12:15] * dt/2) * dt)
    x_new[3:6] += np.dot(M_rot, x[9:12] * dt)
    x_new[6:9] += x[12:15] * dt
    
    print(x_new)
    
    return x_new
         
def state_transition_model_function_jacobian(x, dt):
    global n_dim
    
    F = np.eye(n_dim)
    
    sr = np.sin(x[3])
    sp = np.sin(x[4])
    sy = np.sin(x[5])
    
    cr = np.cos(x[3])
    cp = np.cos(x[4])
    cy = np.cos(x[5])
    
    # x[0]
    F[0][3] = ((cr*sp*cy + sy*sr) * (x[7] + x[13]*dt/2) + (cr*sy - sp*cy*sr)*(x[8] + x[14]*dt/2)) * dt
    F[0][4] = (-cy*sp * (x[6] + x[12]*dt/2) + (cr*sp*cy + sy*sr) * (x[7] + x[13]*dt/2) + (cr*sy + sp*cy*sr)*(x[8] + x[14]*dt/2)) * dt
    F[0][5] = (sy*cp * (x[6] + x[12]*dt/2) - (sr*sp*sy + cy*cr) * (x[7] + x[13]*dt/2) + (sr*cy - sp*sy*cr)*(x[8] + x[14]*dt/2)) * dt
    F[0][6] = cy*cp*dt
    F[0][7] = (sr*sp*cy - sy*cr) * dt
    F[0][8] = (sr*sy + sp*cy*cr) * dt
    F[0][12] = cy*cp*dt**2/2
    F[0][13] = (sr*sp*cy - sy*cr) * dt**2/2
    F[0][14] = (sr*sy+sp*cy*cr)*dt**2/2
    
    # x[1]
    F[1][3] = ((cr*sp*sy - cy*sr)*(x[7] + x[13]*dt**2/2) - (cr*cy + sp*sy*sr)*(x[8] + x[14]**2/2))*dt
    F[1][4] = (-sp*sy*(x[6] + x[12]*dt**2/2) + sr*cp*sy*(x[7] + x[13]*dt**2/2) + cp*sy*cr*(x[8] + x[14]**2/2))*dt
    F[1][5] = (cp*cy*(x[6] + x[12]*dt**2/2) + (sr*sp*cy - sy*cr)*(x[7] + x[13]*dt**2/2) + (sr*sy+sp*cy*cr)*(x[8] + x[14]**2/2))*dt
    F[1][6] = cp*sy*dt
    F[1][7] = (sr*sp*sy + cy*cr)*dt
    F[1][8] = (-sr*cy+sp*sy*cr)*dt
    F[1][12] = cp*sy*dt**2/2
    F[1][13] = (sr*sp*sy + cy*cr)*dt**2/2
    F[1][14] = (-sr*cy+sp*sy*cr)*dt**2/2
    
    # x[2]
    F[2][3] = (cr*cp*(x[7] + x[13]*dt/2) - cp*sr*(x[8] + x[14]*dt/2))*dt
    F[2][4] = (-cp*(x[6] + x[12]*dt/2) - sr*sp*(x[7] + x[13]*dt/2) - sp*cr*(x[8] + x[14]*dt/2))*dt
    F[2][6] = -sp*dt
    F[2][7] = sr*cp*dt
    F[2][8] = cp*cr*dt
    F[2][12] = -sp*dt**2/2
    F[2][13] = sr*cp*dt**2/2
    F[2][14] = cp*cr*dt**2/2
    
    # x[3]    
    F[3][3] = ((cr*sp*cy + sy*sr)*x[10] + (cr*sy - sp*cy*sr)*x[11])*dt
    F[3][4] = (-cy*sp*x[9] + sr*cp*cy*x[10] + cp*cy*cr*x[11])*dt
    F[3][5] = (-sy*cp*x[9] + -(sr*sp*sy + cy*cr)*x[10] + (sr*cy - sp*sy*cr)*x[11])*dt
    F[3][9] = cy*cp*dt
    F[3][10] = (sr*sp*cy - sy*cr)*dt
    F[3][11] = (sr*sy + sp*cy*cr)*dt
    
    # x[4]        
    F[4][3] = ((cr*sp*sy - cy*sr)*x[10] - (cr*cy + sp*sy*sr)*x[11])*dt
    F[4][4] = (-sp*sy*x[9] + sr*cp*sy*x[10] + cp*sy*cr*x[11])*dt
    F[4][5] = (cp*cy*x[9] + (sr*sp*cy - sy*cr)*x[10] + (sr*sy + sp*cy*cr)*x[11])*dt
    F[4][9] = cp*sy*dt
    F[4][10] = (sr*sp*sy + cy*cr)*dt
    F[4][11] = (-sr*cy+sp*sy*cr)*dt
    
    # x[5]        
    F[5][3] = (cr*cp*x[10] - cp*sr*x[11])*dt
    F[5][4] = -(cp*x[9] + sr*sp*x[10] + sp*cr*x[11])*dt
    F[5][5] = 0
    F[5][9] = -sp*dt
    F[5][10] = sr*cp*dt
    F[5][11] = cp*cr*dt
    
    F[6][12] = dt
    F[7][13] = dt
    F[8][14] = dt

    return F

def main(arduino):
    global K, B, Q, R, P, n_dim
    
    # IMU relative position to center of mass of the rocket
    r_imu_1 = np.array([0, 0, 0])
    r_imu_2 = np.array([0, 0, 0])
    
    rotation_matrix_1 = rotation_matrix([0, 0, 0])
    rotation_matrix_2 = rotation_matrix([0, 0, 0])
    inv_rotation_matrix_1 = np.linalg.inv(rotation_matrix_1)
    inv_rotation_matrix_2 = np.linalg.inv(rotation_matrix_2)
    
    # comptute measurement noise
    s_w, s_a, s_pos, s_vel, s_mag, m_w, m_a, m_mag = get_measurement_properties(30)
    
    # Measurement vector :
    # wx :              z[0]
    # wy :              z[1]
    # wz :              z[2]
    # ax :              z[3]
    # ay :              z[4]
    # az :              z[5]
    # x :               z[6]
    # y :               z[7]
    # z :               z[8]
    # vx :              z[9]
    # vy :              z[10]
    # vz :              z[11]
    # mx :              z[12]
    # my :              z[13]
    # mz :              z[14]
    
    state_vector = np.zeros(27)
    n_dim = int(len(state_vector))
    
    Q = np.zeros(n_dim)
    R = np.diag([*s_w, *s_a, *s_pos, *s_vel, *s_mag]) # noise variance  
    
    gravity = np.array([0, 0, -1]) * 9.81
    
    state_vector[18:21] = m_w * np.pi / 180
    state_vector[21:24] = m_a * 9.81 + gravity
    state_vector[15:18] = m_mag

    P = np.random.rand(n_dim, n_dim)
    
    velocity_vector = np.zeros(3)
    orientation_vector = np.zeros(3)
    estimated_angular_angular_acceleration = np.zeros(3)
    last_angular_velocity_measurement = np.zeros(3)
    
    # results logging
    state_vectors = []
    t = []

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
    
    for i in range(500):
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
        gyro1 = np.dot(gyro1, inv_rotation_matrix_1)
        gyro2 = np.dot(gyro2, inv_rotation_matrix_2)
        
        mag1 = np.dot(mag1, inv_rotation_matrix_1)
        mag2 = np.dot(mag2, inv_rotation_matrix_2)
        
        accel1 = np.dot(accel1, inv_rotation_matrix_1)
        accel2 = np.dot(accel2, inv_rotation_matrix_2)
        
        # remove gravity
        M_rocket_rf_to_enu = get_rocket_rotation_matrix(*state_vector[3:6])
        M_enu_to_rocket = np.linalg.inv(M_rocket_rf_to_enu)
        acceleration_measurement = 0.5 * (accel1 + accel2) * 9.81 + np.dot(M_enu_to_rocket, gravity)

        angular_velocity_measurement = 0.5 * (gyro1 + gyro2) * np.pi/180.0
            
        # accel1 = accel1 - np.dot(angular_velocity_measurement, np.dot(angular_velocity_measurement, r_imu_1)) - np.dot(estimated_angular_angular_acceleration, r_imu_1)
        # accel2 = accel2 - np.dot(angular_velocity_measurement, np.dot(angular_velocity_measurement, r_imu_2)) - np.dot(estimated_angular_angular_acceleration, r_imu_2)
        
        
        magnetometer_measurements = 0.5 * (mag1 + mag2)

        # transform GPS measurements to NEU frame
        
        # exec kalman filter on measurements depending on measurements
        state_vector = kalman_gps(state_vector, angular_velocity_measurement, acceleration_measurement, gps, magnetometer_measurements, dt)
        
        # integrate measurements
        velocity_vector = np.array([state_vector[6], state_vector[7], state_vector[8]])
        orientation_vector = np.array([state_vector[9], state_vector[10], state_vector[11]])
                
        state_vectors.append(state_vector)
        t.append(t_end)
        
        # compute angular acceleration
        if dt != 0: estimated_angular_angular_acceleration = (angular_velocity_measurement - last_angular_velocity_measurement)/dt
        else: estimated_angular_angular_acceleration = np.zeros(3)
        
        last_angular_velocity_measurement = angular_velocity_measurement
        last_acceleration_measurement = acceleration_measurement
    
    
    legend = ["x", "y", "z", "angx", "angy", "angz", "vx", "vy", "vz", "wx", "wy", "wz", "ax", "ay", "az", "mx", "my", "mz", "bwx", "bwy", "bwz", "bax", "bay", "baz", "bmx", "bmy", "bmz"]   
    state_vectors = np.array(state_vectors).transpose()
    n = int(len(state_vectors))
    fig = plt.figure(figsize=[3, 15], dpi=300)
        
    axes = fig.subplots(n, 1, sharex = True)
    
    for i, ax in enumerate(axes):
        ax.set_ylabel(legend[i])
        ax.plot(t, state_vectors[:][i])
    
    
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

