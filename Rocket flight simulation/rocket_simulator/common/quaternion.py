# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 13:07:05 2022

@author: Thomas Maynadié
"""
import numpy as np
from scipy.linalg import expm

def euler_to_dcm(roll, pitch, yaw):
    cy = np.cos(yaw*np.pi/180);
    sy = np.sin(yaw*np.pi/180);
    cp = np.cos(pitch*np.pi/180);
    sp = np.sin(pitch*np.pi/180);
    cr = np.cos(roll*np.pi/180);
    sr = np.sin(roll*np.pi/180);
    
    M = np.array([
        [cp*cy, -cr*sy + sr*sp*cy,  sr*sy + cr*sp*cy],
        [cp*sy,  cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy],
        [-sp, sr*cp, cr*cp]])
    
    return M

def angular_rate_matrix(omega):
    wx, wy, wz = omega

    return np.array(
        [[0,     -wx,    -wy,    -wz],
        [ wx,     0,      wz,    -wy],
        [ wy,    -wz,     0,      wx],
        [ wz,     wy,    -wx,     0 ]], dtype=float
        )

class Quaternion():
    def __init__(self, *args, deg=False):
        if deg is True:
            conv_factor = np.pi/180.0
        else:
            conv_factor = 1.0
        
        if (len(args) == 4):
            self.q = np.array([args[0], args[1], args[2], args[3]])
            
        elif (len(args) == 3):
            args = np.array(args)*conv_factor
            
            cy = np.cos(args[2] * 0.5);
            sy = np.sin(args[2] * 0.5);
            cp = np.cos(args[1] * 0.5);
            sp = np.sin(args[1] * 0.5);
            cr = np.cos(args[0] * 0.5);
            sr = np.sin(args[0] * 0.5);
        
            q0 = cr * cp * cy + sr * sp * sy;
            q1 = sr * cp * cy - cr * sp * sy;
            q2 = cr * sp * cy + sr * cp * sy;
            q3 = cr * cp * sy - sr * sp * cy;
            
            self.q = np.array([q0, q1, q2, q3])
            
        else:    
            self.q = np.array([1, 0, 0, 0]) 
    
    def convert_to_attitude_vector(self, deg=False):
        q = self.q
        
        alpha = q[0]*q[2] - q[3]*q[1];

        if deg is True:
            conv_factor = 180.0/np.pi
        else:
            conv_factor = 1.0
        
        if abs(alpha) == 0.5:
            pitch = np.sign(2*alpha) * np.pi / 2 * conv_factor
            roll = 0.0
            yaw = 2 * np.sign(alpha) * np.arctan2(q[1],q[0])*conv_factor

        else:
            pitch = np.arcsin(2*alpha)*conv_factor
            roll = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))*conv_factor
            yaw = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))*conv_factor
 
        return np.array([roll, pitch, yaw])
    
    def __add__(self, b):
        return Quaternion(*(self.get_coefficients() + b.get_coefficients()))
        
    def __mul__(self, b):
        q1_coef = self.get_coefficients()
        
        a1 = q1_coef[0]
        b1 = q1_coef[1]
        c1 = q1_coef[2]
        d1 = q1_coef[3]
        
        if (type(b) is type(self)):
            q2_coef = b.get_coefficients()
            
            a2 = q2_coef[0]
            b2 = q2_coef[1]
            c2 = q2_coef[2]
            d2 = q2_coef[3]
            
            q0 = a1*a2 - b1*b2 - c1*c2 - d1*d2
            q1 = a1*b2 + b1*a2 + c1*d2 - d1*c2
            q2 = a1*c2 - b1*d2 + c1*a2 + d1*b2
            q3 = a1*d2 + b1*c2 - c1*b2 + d1*a2
            
            return Quaternion(q0, q1, q2, q3)
            
        if (type(b) is type(1) or type(b) is type(1.0)): 
            return Quaternion(*(b*self.get_coefficients()))
        
    def __rmul__(self, b):
        ''' multiply other with self, b * Foo() '''
        
        q1_coef = self.get_coefficients()
        
        a1 = q1_coef[0]
        b1 = q1_coef[1]
        c1 = q1_coef[2]
        d1 = q1_coef[3]
        
        if (type(b) is type(self)):
            q2_coef = b.get_coefficients()
            
            a2 = q2_coef[0]
            b2 = q2_coef[1]
            c2 = q2_coef[2]
            d2 = q2_coef[3]
            
            q0 = a1*a2 - b1*b2 - c1*c2 - d1*d2
            q1 = a1*b2 + b1*a2 + c1*d2 - d1*c2
            q2 = a1*c2 - b1*d2 + c1*a2 + d1*b2
            q3 = a1*d2 + b1*c2 - c1*b2 + d1*a2
            
            return Quaternion(q0, q1, q2, q3)
            
        if (type(b) is type(1) or type(b) is type(1.0)): 
            return Quaternion(*(b*self.get_coefficients()))
        
    def conjugate(self):
        return Quaternion(self.q[0], -self.q[1], -self.q[2], -self.q[3])
    
    def get_coefficients(self):
        return self.q
        
    def DCM(self):        
        q0, q1, q2, q3 = self.q
                
        return np.array([
            [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])
        
            
    def norm2(self):
        return self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2

    def normalize(self):
        q_norm = np.sqrt(self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2)

        self.q = self.q/q_norm

        return Quaternion(*self.q)

    def propagate(self, omega, dt):
        inst_angular_vel_matrix = angular_rate_matrix(omega)

        q_new = self.q + np.dot(inst_angular_vel_matrix*dt, self.q)

        return Quaternion(*q_new).normalize()