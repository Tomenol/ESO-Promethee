import logging

import matplotlib.pyplot as plt
import numpy as np

import rocket_simulator
from rocket_simulator import Quaternion 
from rocket_simulator import earth

"""
Experimental rocket simulation example :

This short scipt shows one of the possible applications for the rocket simulator toolbox
"""

def simulate_launch():
	# logging 
	logging.basicConfig(format='%(asctime)s - %(levelname)s -> %(message)s', level=logging.INFO)

	# -------------------------------------
	# 	   setup simulation parameters
	# -------------------------------------

	# launch date
	year = 2022
	day_year = 191/365.25 # 10th july = 191th day of the year
	time = (11.0 + 25.0/60.0)/24.0/365.25 # 11:25 am

	epoch = year + day_year + time
	initial_time = 0
	launch_time = 5
	t_end = 60

	# resolution
	dt = 0.01
	npoints = int((t_end-initial_time)/dt)

	# Launch parameters
	initial_rpy = np.array([0, -85, 0])

	# setup simulation modules
	env = rocket_simulator.Environment(epoch, logger=logging)
	# pad = rocket_simulator.LaunchPad(43.2083, -0.0615, 0, env, initial_attitude_euler=initial_rpy, deg=True, logger=logging)
	pad = rocket_simulator.LaunchPad(43.2083, -0.0615, 0.1, env, initial_attitude_euler=initial_rpy, deg=True, logger=logging)

	# -------------------------------------
	# 		setup rocket parameters
	# -------------------------------------

	# inertial/geometric properties
	mass = 5.5
	R = 11.6e-2/2
	length = 2.097
	e = 3e-3

	Ix_y = mass/12 * (3*((R + e)**2 + R**2) + length**2)
	Iz = mass/2 * ((R + e)**2 + R**2)
	inertia_matrix = np.array([
		[Ix_y, 	0, 		0,],
		[0, 	Ix_y, 	0],
		[0, 	0, 		Iz],
		])

	# engine properties
	thrust_law 	= np.array([0.0, 891.9, 845.5, 799.06, 739.59, 396.13, 0.00])
	t_law 		= np.array([0.0, 0.060, 0.280, 0.5000, 1.0000, 3.0000, 3.8])
	delta 		= 0.01*np.pi/180
	thrust_dir 	= np.array([np.cos(delta), np.sin(delta), 0])

	
	# Aerodynamic properties 
	tip_length = 0.087 
	h_emp = 0.235
	base_len = 0.158

	Sref_tail = 0.5*(base_len + tip_length)*h_emp
	Sref_body = np.pi*R**2

	alpha0 = 0.0

	lift_rate_coefficient_body 		= 0.15
	drag_coefficient_body 			= 0.14

	lift_rate_coefficient_tail 		= 2*np.pi
	drag_coefficient_tail 			= 0.05

	moment_coefficent_rate_body 	= 0
	moment_coefficent_rate_tail 	= 0

	position_aerodynamic_center_body=0.75*length
	position_aerodynamic_center_tail=0.285
	position_center_of_thrust=0
	position_cog=0.8145

	# set up main simulation
	rocket = rocket_simulator.RocketSimulator( 
			initial_time, 
			launch_time, 
			t_end, 
			pad, 
			env, 
			mass, 
			inertia_matrix, 
			alpha0, 
			Sref_body, 
			Sref_tail, 
			length, 
			0.5*(base_len + tip_length), 
			lift_rate_coefficient_body, 
			drag_coefficient_body, 
			lift_rate_coefficient_tail, 
			drag_coefficient_tail, 
			moment_coefficent_rate_body,
			moment_coefficent_rate_tail,
			position_aerodynamic_center_body=position_aerodynamic_center_body,
			position_aerodynamic_center_tail=position_aerodynamic_center_tail,
			position_center_of_thrust=position_center_of_thrust,
			position_cog=position_cog,
			logger=logging,
			)
	rocket.set_motor_properties(thrust_law, t_law, thrust_dir)

	print(rocket)

	# -------------------------------------
	# 			Run simulation
	# -------------------------------------

	rocket.propagate(t_end, n_points=npoints)

	np.save("sim_rocket_states.npy", rocket.states)
	np.save("sim_rocket_t.npy", rocket.t)
	np.save("sim_rocket_w.npy", rocket.w)

	# -------------------------------------
	# 			Process results
	# -------------------------------------

	# plot simulation results : rocket states
	rocket.plot_states()

	# fig = plt.figure()
	# ax = fig.subplots(3, 1)
	# ax[0].plot(rocket.t, rocket.w[0])
	# ax[1].plot(rocket.t, rocket.w[1])
	# ax[2].plot(rocket.t, rocket.w[2])

	plt.show()

	

if __name__=="__main__":
	simulate_launch()