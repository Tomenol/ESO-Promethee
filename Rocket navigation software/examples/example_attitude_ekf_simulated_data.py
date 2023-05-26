# -*- coding: utf-8 -*-
"""
Created on Wed Jan 26 15:39:21 2022

@author: Thomas Maynadié
"""
import numpy as np
import matplotlib.pyplot as plt
import pathlib

import navigation_system.prefilters.imu_prefilters as imu_prefilters
import navigation_system.ekf.navigation_ekf as navigation_ekf

import navigation_system.helpers.simulation.rotation_simulation as rotation_simulation

from navigation_system.helpers.quaternion_helper import quaternion

data_path = pathlib.Path('.').resolve()

def main_routine():
	# initialization
	# get sensor parameters from the first 30 still measurements

	# --------------------------------------------
	#               sensor definition
	# --------------------------------------------

	# 1st MARG sensor
	# gyroscope
	gyroscope_std_deviation_1 = np.array([0.1, 0.12, 0.098])*5
	gyroscope_bias_1 = np.array([0.2, 0.15, 0.12])*0.1

	# accelerometer
	accelerometer_std_deviation_1 = np.array([0.072, 0.078, 0.069])*1
	accelerometer_bias_1 = np.array([0.03, 0.01, 0.02])*0.2

	# magnetometer
	magnetometer_std_deviation_1 = np.array([0.2, 0.21, 0.19])*1
	magnetometer_bias_1 = np.array([0.11, 0.15, 0.12])*0.01

	# 2nd MARG sensor
	# gyroscope
	gyroscope_std_deviation_2 = np.array([0.1, 0.12, 0.098])
	gyroscope_bias_2 = np.array([0.2, 0.15, 0.12])

	# accelerometer
	accelerometer_std_deviation_2 = np.array([0.072, 0.078, 0.069])
	accelerometer_bias_2 = np.array([0.03, 0.01, 0.02])

	# magnetometer
	magnetometer_std_deviation_2 = np.array([0.2, 0.21, 0.19])
	magnetometer_bias_2 = np.array([0.11, 0.15, 0.12])

	# Pressure sensor
	pressure_std_deviation = 0.01
	pressure_bias = 10

	temperature_std_deviation = 0.01
	temperature_bias = 0.1

	# --------------------------------------------
	#            Component defintion
	# --------------------------------------------

	# Gyroscope pre-filter
	gyroscope_prefilter = imu_prefilters.gyroscope_preprocessor(gyroscope_std_deviation_1, gyroscope_bias_1, gyroscope_std_deviation_2, gyroscope_bias_2)

	# Magnetometer + Accelerometer preprocessing unit
	accelerometer_prepreprocessor = imu_prefilters.imu_sensor_preprocessor(accelerometer_std_deviation_1, accelerometer_bias_1, accelerometer_std_deviation_2, accelerometer_bias_2)
	magnetometer_prepreprocessor = imu_prefilters.imu_sensor_preprocessor(magnetometer_std_deviation_1, magnetometer_bias_1, magnetometer_std_deviation_2, magnetometer_bias_2)

	accelerometer_processed_std_deviation = accelerometer_prepreprocessor.get_std_deviation()
	accelerometer_processed_bias = accelerometer_prepreprocessor.get_bias()

	gyroscope_processed_std_deviation = gyroscope_prefilter.get_std_deviation()
	gyroscope_processed_bias = gyroscope_prefilter.get_bias()

	magnetometer_processed_std_deviation = magnetometer_prepreprocessor.get_std_deviation()
	magnetometer_processed_bias = magnetometer_prepreprocessor.get_bias()

	print(gyroscope_processed_std_deviation)

	# GPS preprocessor
	gps_pos_std_deviation = np.zeros(3)
	gps_vel_std_deviation = np.zeros(3)
	# TODO

	# --------------------------------------------
	#               Constants
	# --------------------------------------------

	# All global quantities are given in the ENU reference frame
	mag_declination = 0

	# --------------------------------------------
	#               Rocket geometry
	# --------------------------------------------

	# All local quantities are given in the Rocket Frame (RF) reference frame (x : main rocket axis / y, z : arbitrarly chosen with respect to the navigation module's geometry)

	# Navigation module position
	r_rocket_cog_to_nav_ref = np.array([0.55, 0, 0]) # tbd with structural team

	# MARGs position
	r_nav_ref_to_marg1 = np.array([0.1, -1, 0]) # tbd with structural team
	r_nav_ref_to_marg2 = np.array([0.1, -1, 0]) # tbd with structural team

	# --------------------------------------------
	#             Other variables
	# --------------------------------------------
	iteration_nb = 300

	data = np.load(data_path / "sim_rocket_states.npy")[:, ::100]
	omega_body = np.load(data_path / "sim_rocket_w.npy")[:, ::100]
	t = np.load(data_path / "sim_rocket_t.npy")[::100]

	t0 = 0
	tf = 30
	dt = (tf - t0)/iteration_nb

	# simulation
	initial_attitude = np.array([0, 0, 0])
	initial_position = np.array([0, 0, 0])

	angular_rate = np.array([1.0, 0.0, 2.0])
	accel_enu = np.array([0, 0.0, 0.0])

	simulation = rotation_simulation.rotation_sim_quaternions(initial_attitude, angular_rate, accel_enu, tf, iteration_nb, data[0:3,0], frame="body")

	# results
	time_array, theoretical_trajectory = simulation.get_trajectory()
	estimated_trajectory = [[*quaternion(*initial_attitude).get_coefficients(), 0, 0, 0, 0, 0, 0, 0, 0, 0]]
	time_array = np.linspace(t0, tf, iteration_nb)

	w_m = [angular_rate]

	# --------------------------------------------
	#             Kalman Variables
	# --------------------------------------------
	navigation_state_estimator = navigation_ekf.NavigationEKF(initial_attitude, initial_position, gyroscope_std_deviation_1, gyroscope_bias_1, accelerometer_std_deviation_1, accelerometer_bias_1, magnetometer_std_deviation_1, magnetometer_bias_1, gps_pos_std_deviation, gps_vel_std_deviation, pressure_std_deviation, pressure_bias, temperature_std_deviation, temperature_bias, mag_declination)

	# --------------------------------------------
	#               MAIN LOOP
	# --------------------------------------------
	measurements = []

	for i in range(1, iteration_nb):
	# get measurement 
		raw_accelerometer_measurement_1, raw_accelerometer_measurement_2, raw_gyroscope_measurement_1, raw_gyroscope_measurement_2, raw_magnetometer_measurement_1, raw_magnetometer_measurement_2, raw_pressure_measurement, raw_gps_measurement = simulation.measurement(accelerometer_std_deviation_1, accelerometer_std_deviation_2, accelerometer_bias_1, accelerometer_bias_2, gyroscope_std_deviation_1, gyroscope_std_deviation_2, gyroscope_bias_1, gyroscope_bias_2, magnetometer_std_deviation_1, magnetometer_std_deviation_2, magnetometer_bias_1, magnetometer_bias_2, mag_declination, i)

		# preprocess measurements
		processed_accelerometer_measurement = accelerometer_prepreprocessor.process_measurements(raw_accelerometer_measurement_1, raw_accelerometer_measurement_2)
		processed_gyroscope_measurement = gyroscope_prefilter.process_measurements(raw_gyroscope_measurement_1, raw_gyroscope_measurement_2, dt)
		processed_magnetometer_measurement = magnetometer_prepreprocessor.process_measurements(raw_magnetometer_measurement_1, raw_magnetometer_measurement_2)

		# Kalman step
		measurement_vector = np.array([*raw_accelerometer_measurement_1, *raw_magnetometer_measurement_1])
		input_vector = raw_gyroscope_measurement_1
		estimated_trajectory.append(navigation_state_estimator.filter_step(input_vector, measurement_vector, dt))

	measurements = np.asfarray(measurements).T
	print(measurements)
	print("measurements")
	print(np.mean(measurements[0:3], axis=1))
	print(np.var(measurements[0:3], axis=1)**0.5)

	print(np.mean(measurements[3:6], axis=1))
	print(np.var(measurements[3:6], axis=1)**0.5)

	print(np.mean(measurements[6:9], axis=1))
	print(np.var(measurements[6:9], axis=1)**0.5)

	# --------------------------------------------
	#               POST PROCESSING
	# --------------------------------------------
	estimated_trajectory = np.asfarray(estimated_trajectory).transpose()
	theoretical_trajectory = np.asfarray(theoretical_trajectory).transpose()
	w_m = np.array(w_m).transpose()

	fig = plt.figure()
	subplots = fig.subplots(13, 1)

	for i in range(13):
		if i < len(subplots)-1:
			plt.setp(subplots[i].get_xticklabels(), visible=False)


	y_labels = ["q0", "q1", "q2", "q3", "b_wx", "b_wy", "b_wz", "b_ax", "b_ay", "baz", "b_mx", "b_my", "bmz"]

	for i in range(4):
		subplots[i].plot(time_array, theoretical_trajectory[i], "--r", linewidth=1)
		subplots[i].plot(time_array, estimated_trajectory[i], "b", linewidth=1)

		subplots[i].set_xlabel("t [s]")
		subplots[i].set_ylabel(y_labels[i])
		subplots[i].grid()
	    
	for i in range(3):
		subplots[i+4].plot(time_array, np.ones(iteration_nb)*gyroscope_bias_1[i], "--r", linewidth=1)
		subplots[i+4].plot(time_array, estimated_trajectory[i+4], "b", linewidth=1)

		subplots[i+7].plot(time_array, np.ones(iteration_nb)*accelerometer_bias_1[i], "--r", linewidth=1)
		subplots[i+7].plot(time_array, estimated_trajectory[i+7], "b", linewidth=1)

		subplots[i+10].plot(time_array, np.ones(iteration_nb)*magnetometer_bias_1[i], "--r", linewidth=1)
		subplots[i+10].plot(time_array, estimated_trajectory[i+10], "b", linewidth=1)

	print("done")
	plt.show()

if __name__ == '__main__':
	main_routine()