import matplotlib.pyplot as plt
import numpy as np
from scipy import integrate, interpolate

def compute_aerodynamic_force(vel, rho, S, Cx, Cza, omega, r):
	V_apparent = vel + np.cross(omega, r)

	V_apparent_norm = np.linalg.norm(V_apparent)
	alpha_apparent = np.arctan(V_apparent[2]/V_apparent[0])
	alpha = np.arctan(vel[2]/vel[0])

	q = 0.5*rho*V_apparent_norm**2
	Fz = q*S*Cza*alpha_apparent
	Fx = q*S*Cx
	# print("alpha_apparent, ", alpha_apparent)
	# print("alpha, ", alpha)

	M = aero_to_body_matrix(alpha)

	return np.dot(M, np.array([-Fx, 0, -Fz]))

def aero_to_body_matrix(alpha):
	return np.array([
		[ np.cos(alpha), 	0, 	 	-np.sin(alpha)	],
		[ 0, 				1, 		 0 				],
		[ np.sin(alpha), 	0, 		 np.cos(alpha)	]], dtype=float)

def global_to_body_matrix(theta):
	return np.array([
		[ np.cos(theta), 	0.0, 	 	-np.sin(theta)	],
		[ 0.0, 				1.0, 		 0.0 				],
		[ np.sin(theta), 	0.0, 		 np.cos(theta)	]], dtype=float)

rho = 1.225

Cza = 2*np.pi
Cx = 0.05

I = 0.05

S = 0.25*0.15

V_aero = np.array([10, 0, 0])

t0 = 0
tf = 40
dt = 0.01
r = 0.3

N = int((tf-t0)/dt)

x = np.zeros((2))
x[0] = 15*np.pi/180 # alpha

t_transition = tf/2

def state_transition_model_function(t, x):
	theta 		= x[0]
	d_theta 	= x[1]

	if t < t_transition:
		alpha_0 = 0
	else:
		alpha_0 = 15*np.pi/180

	V = np.dot(aero_to_body_matrix(alpha_0), V_aero)

	omega = d_theta*np.array([0, 1, 0])
	r_vec = -np.array([r, 0, 0])

	V_body = np.dot(global_to_body_matrix(theta), V)
	# print(V_body)

	F = compute_aerodynamic_force(V_body, rho, S, Cx, Cza, omega, r_vec)
	# print("F, ", F)
	M = np.cross(r_vec, F)
	# print("M, ", M)

	M = np.dot(global_to_body_matrix(theta).T, M)

	dx = x.copy()
	dx[0] = d_theta
	dx[1] = M[1]/I

	return dx

solution = integrate.solve_ivp(state_transition_model_function, (t0, tf), x, method='Radau', rtol=1e-3)

t = np.arange(0, N*dt, dt)
interp_states = interpolate.interp1d(solution["t"], solution["y"])(t)

fig = plt.figure()
axes = fig.subplots(2, 1, sharex=True)

labels = ["alpha [rad]", "d_alpha [rad/s]", "dd_alpha [rad/s2]"]

for i in range(2):
	axes[i].plot(t, interp_states[i], "-r")

	axes[i].set_ylabel(labels[i])
	axes[i].set_xlabel("t [s]")

	axes[i].grid()

plt.show()

fig = plt.figure()
axes = fig.subplots(3, 1, sharex=True)

labels = ["alpha [rad]", "d_alpha [rad/s]", "dd_alpha [rad/s2]"]