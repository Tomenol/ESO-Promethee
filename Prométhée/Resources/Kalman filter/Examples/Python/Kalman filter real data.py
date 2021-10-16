"""
This python script is used to compute the Kalman filter state vector estimate using real IMU measurements (still)
combined with a virtual GPS.

this script is part of ESO's Promethee project.

Version : 16-10-2021

@author: Thomas Maynadié
"""

import numpy as np
import matplotlib.pyplot as plt

# import real experimental data
data_filename = "Experimental data//log_experimental_data_5102021_16_23_29.fusex_dat"
data = np.genfromtxt(data_filename, skip_header=1, dtype=["float","float","float","float","float","float", "float","float","float","float","float","float", "float"], names=['t', 'ax1','ax2','ay1','ay2','az1','az2', 'wx1', 'wx2', 'wy1', 'wy2', 'wz1', 'wz2'])

acceleration_x = data["ax1"]
sigma_a = np.sqrt(np.var(acceleration_x))

t = data["t"]

# we create a virtual gps fixed at x=0
sigma_gps = 1
gps_dat = np.random.normal(0, sigma_gps, len(t))

def kalman_gps(z_gps, a, x_hat, P_hat, dt):    
    """Used to compute the estimated state vector (Kalman) using the IMU + GPS (virtual) measurements
    
    Inputs :
        - z_gps : GPS measurements
        - a : accelerometer measurements
        - x_hat : actual state vector
        - P_hat : actual estimate covariance matrix
        - dt : time step between two estimates
        
    Returns :
        - x_hat : estimated state vector
        - P_hat : updated estimate covariance matrix
    """
    
    n = np.size(x_hat)
    
    z = np.matrix([z_gps]).T
    u = np.matrix([a]).T

    F = np.matrix([[1, dt, 0, 0], 
                   [0, 1, -dt, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    
    B = np.matrix([0, dt, 0, 0]).T
    
    R = np.diag([sigma_gps**2])
    H = np.matrix([[1, 0, 0, 0]])

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

def kalman_no_gps(a, x_hat, P_hat, dt):    
    """Used to compute the estimated state vector (Kalman) using only the IMU measurements
    
    Inputs :
        - a : accelerometer measurements
        - x_hat : actual state vector
        - P_hat : actual estimate covariance matrix
        - dt : time step between two estimates
        
    Returns :
        - x_hat : updated state vector
        - P_hat : updated estimate covariance matrix
    """
    
    n = np.size(x_hat)
    u = np.matrix([a])

    F = np.matrix([[1, dt, 0, 0], 
                   [0, 1, -dt, 0],
                   [0, 0, 1, 0], 
                   [0, 0, 0, 1]])
    
    B = np.matrix([0, dt, 0, 0]).T
    Q = B.T*B * sigma_a ** 2
    
    # predict
    x_hat = F*x_hat + B * u
    P_hat = F*P_hat*F.T + Q

    return x_hat, P_hat

def main():
    global acceleration_x
    
    last_t = 0
    
    pos_kalman = []
    
    # initialization of position and velocity estimation without Kalman filtering
    vel_no_kalman = []
    pos_no_kalman = []

    vel_estimation_no_kalman = 0
    pos_estimation_no_kalman = 0

    # accelerometer bias correction
    acceleration_x = acceleration_x - np.mean(acceleration_x[1:]) + 0.01
    acceleration_x[0] = 0
    
    plt.plot(acceleration_x)
    
    # Kalman filter initialization 
    x_est = np.matrix([0, 0, 0, 0]).T
    P_est = np.matrix(np.zeros((4, 4)))
    
    # compute the Kalman state estimate for each sample point
    for i, _ax in enumerate(acceleration_x):
        dt = (t[i] - last_t) * 1e-6
        last_t = t[i]        
        
        # compute position estimation without Kalman
        vel_estimation_no_kalman += _ax * dt
        pos_estimation_no_kalman += vel_estimation_no_kalman * dt
    
        gps_x = gps_dat[i]
        
        # we suppose here that the gps has 1/10th the sample rate of the accelerometer
        if (i%10 == 0):
            x_est, P_est = kalman_gps(gps_x, _ax, x_est, P_est, dt)
        else:
            x_est, P_est = kalman_no_gps(_ax, x_est, P_est, dt)
        
        pos_no_kalman.append(pos_estimation_no_kalman)   
        pos_kalman.append(float(x_est[0]))
      
    # plot results
    fig = plt.figure(dpi=300)
    axs = fig.add_subplot(111)
    
    axs.plot(t * 1e-6, pos_kalman)        
    axs.plot(t * 1e-6, pos_no_kalman)  
    
    axs.legend(["Kalman", "Accelerometer only"])
    axs.grid()
    
    axs.set_title("1D Position estimate")
    
if __name__ == '__main__':
    main()