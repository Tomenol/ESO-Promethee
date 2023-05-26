import numpy as np
from rocket_simulator import rocket
from rocket_simulator.quaternion import Quaternion

alpha = -45 
beta = -45

print("alpha : ", alpha)
print("beta : ", beta)

alpha = alpha * np.pi/180
beta = beta * np.pi/180

vel_aero = np.array([100, 0, 0])
vel_body = rocket.aero_to_body_frame(vel_aero, alpha, beta)

print("aerodynamic frame : ", vel_aero)
print("body frame : ", vel_body)

vel_norm = np.linalg.norm(vel_body)
alpha_est = np.arcsin(vel_body[2]/vel_norm)
beta_est = np.arctan2(vel_body[1], vel_body[0])

print("alpha estimation : ", alpha_est * 180/np.pi)
print("beta estimation : ", beta_est * 180/np.pi)
