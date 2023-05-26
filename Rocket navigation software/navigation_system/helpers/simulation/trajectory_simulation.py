# -*- coding: utf-8 -*-
"""
Created on Thu Jan 13 11:37:20 2022

@author: Thomas Maynadié
"""
import numpy as np
import scipy.integrate
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d

class trajectory_sim:
    def __init__(self, initial_attitude_ENU, t0, t_stop_engine, tf, dt):     
        # x = [pos, ang, vel, w]
        self.t0 = t0
        self.t_stop_engine = t_stop_engine
        self.tf = tf
        
        self.x0 = np.array([*np.zeros(3), *(initial_attitude_ENU * np.pi/180), *np.zeros(3), *np.zeros(3)])
        
        self.x_traj = []
        self.t = []
        
        res = scipy.integrate.solve_ivp(self.model_transition_function_boost, [t0, t_stop_engine], self.x0, method="Radau", t_eval=np.linspace(t0, t_stop_engine, int((t_stop_engine-t0)/dt)))
        
        self.x_traj = [*self.x_traj, *np.transpose(res["y"])]
        self.t = [*self.t, *res["t"]]
        
        res = scipy.integrate.solve_ivp(self.model_transition_function_ballistic, [t_stop_engine, tf], self.x_traj[len(self.x_traj)-1], method="RK45", atol=1e-6, rtol=1e-6, t_eval=np.linspace(t_stop_engine, tf, int((tf-t_stop_engine)/dt)), events=self.ground_hit)
        
        self.x_traj = [*self.x_traj, *np.transpose(res["y"])]
        self.t = [*self.t, *res["t"]]
    
    def ground_hit(self, t, x):
        return x[2]
        
    def get_measurements(self, n, s_accel, s_pos, s_vel, s_gyro, s_mag, m_w, m_a):
        if n>len(self.t): return 0
        else:
            M = np.linalg.inv(self.get_rotation_matrix(np.array(self.x_traj[n][3:6])))
            
            g = np.array([0, 0, -1])
            
            w = self.x_traj[n][9:12] + np.random.normal(0, np.sqrt(s_gyro), 3) * 180/np.pi + m_w
            a = ((self.x_traj[n][6:9] - self.x_traj[n-1][6:9])/(self.t[n] - self.t[n-1]) + np.random.normal(0, np.sqrt(s_accel), 3))/9.81 - g + m_a
            mag = np.array([0, 1, 0]) + np.random.normal(0, np.sqrt(s_mag), 3)
            
            pos = self.x_traj[n][0:3] + np.random.normal(0, np.sqrt(s_pos), 3)
            vel = self.x_traj[n][6:9] + np.random.normal(0, np.sqrt(s_vel), 3)
            
            gps = np.array([*pos, *vel])
            
            w = np.dot(M, w)
            a = np.dot(M, a)
            mag = np.dot(M, mag)
            
            t = self.t[n]
        
        return t, a, w, gps, mag
    
    def get_trajectory_position(self):
        traj = np.transpose(self.x_traj)
        
        return self.t, traj[0], traj[1], traj[2]
    
    def get_rotation_matrix(self, theta):
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
        
    def model_transition_function_boost(self, t, x):
        r = x[0:3]
        v = x[6:9]
        theta = x[3:6]
        w = x[9:12]
        
        r_engine = np.array([-0.75, 0, 0])
        J = 5e-2
        M = 2
        
        a_t = (8*t**0.2/(0.2*t+0.5)+1) * 9.81
        
        # compute trajectory in ENU frame
        F_g = -9.81 * M * np.array([0, 0, 1])*0 # gravity   
        
        F_t = a_t * M * np.array([1, 0, 0]) # thrust towards +x in rocket RF
        F_t = np.dot(self.get_rotation_matrix(theta), np.dot(self.get_rotation_matrix(np.array([0.0002, 0.00015, 0])), F_t)) # engine vibrations
        
        print(t)
        print(F_t)
        
        C_t = np.cross(r_engine, F_t) # moment
        
        F = F_t + F_g
        
        r_dot = v
        v_dot = F/M
        
        w_dot = (C_t - 80e3*w**2) / J
        theta_dot = w
        
        return np.array([*r_dot, *theta_dot, *v_dot, *w_dot])    
    
    def model_transition_function_ballistic(self, t, x):
        r = x[0:3]
        v = x[6:9]
        theta = x[3:6]
        w = x[9:12]
        
        r_engine = np.array([-1, 0, 0])
        J = 5e-2
        M = 30
        
        # compute trajectory in ENU frame
        F_g = -9.81 * M * np.array([0, 0, 1]) # gravity
        
        r_dot = v
        v_dot = F_g/M
        
        w_dot = np.zeros(3)
        theta_dot = w
            
        return np.array([*r_dot, *theta_dot, *v_dot, *w_dot])    


# initial_attitude_ENU = np.array([0, -90, 0])
# traj = trajectory_sim(initial_attitude_ENU, 0, 3, 60, 0.01)
# t, x, y, z = traj.get_trajectory_position()

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# # ax.set_xlim([-10, 10])
# # ax.set_ylim([-10, 10])
# # ax.set_zlim([0, 100])

# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")

# ax.plot(x, y, z)

# print("ok")