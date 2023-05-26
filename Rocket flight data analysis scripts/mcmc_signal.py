import numpy as np
import matplotlib.pyplot as plt

from datetime import datetime
from sys import stdout

class MCMCSignal(object):
	def __init__(self, signal_true):
		self.theoretical_measurement = signal_true

	def measurement_model(self, theta, n_points):
		b, sigma = theta

		signal_measured = self.theoretical_measurement + b

		return signal_measured

	def log_likelyhood(self, theta, data, sigma_signal):
		th_data  = self.measurement_model(theta, len(data))

		logl = np.sum(-0.5*((th_data - data)**2/(sigma_signal**2 + theta[1]**2)))
		return logl

	def generate_chains(self, data, sigma_signal, n_points=10000):
		mean = np.mean(data) - self.theoretical_measurement
		sigma = np.sqrt(np.var(data))

		chains = np.ndarray((2, n_points), dtype=float)
		logP_arr = np.ndarray((n_points,), dtype=float)

		theta = np.array([mean, sigma])
		logP = self.log_likelyhood(theta, data, sigma_signal)

		print("\n\nMCMC Algorithm :\n")
		t_last = 0.0
		dt_mean = 0.0
		dt = 0.0

		n = 0
		while n < n_points:
			accept = False

			theta_new = theta + np.random.multivariate_normal(np.zeros(2), np.diag([1, 1]))*1e-4

			if theta_new[1] > 0:
				logP_new = self.log_likelyhood(theta_new, data, sigma_signal)*1.0

				if logP_new > logP:
					accept = True
				else:
					logU = np.log(np.random.rand())

					if logP_new - logP > logU:
						accept = True

			if accept is True:
				logP = logP_new
				theta = theta_new

				logP_arr[n] = logP
				chains[:, n] = theta

				n += 1

				dt = abs(datetime.now().microsecond - t_last)
				t_last = datetime.now().microsecond


				if n == 0:
					h = -1
					m = -1
				else:
					dt_mean = (dt_mean * (n-1) + dt)/n
					t_remaining = (dt_mean * (n_points - n))/1e6

					h = int(t_remaining/3600)
					m = np.mod(t_remaining/60, 60)

				stdout.flush()
				stdout.write("\r                                                                                     ")
				stdout.write("\rstep : %d/%d       -  %d hour(s) and %d minute(s) remaining..." % (n, n_points, int(h), int(m)))
				stdout.flush()

		return chains, logP_arr



acceleration_true = np.array([0, 0, -9.81])
acceleration_measurement_sigma = 0.01

acceleration_bias = np.array([1.5, 0.08, 0.97])
acceleration_sigma = np.array([0.05, 0.08, 0.06])

n_measurements = 100

mcmc = MCMCSignal(acceleration_true[0])
acceleration_measurement = mcmc.measurement_model((acceleration_bias[0], acceleration_sigma[0]), n_measurements) + np.random.normal(0, acceleration_measurement_sigma, n_measurements)
print(acceleration_measurement)

chains, logP_arr = mcmc.generate_chains(acceleration_measurement, acceleration_measurement_sigma, n_points=100000)

np.save("chains_accel_test.npy", chains)

fig = plt.figure()
axes = fig.subplots(4, 1)
labels = ["b", "s"]

for i in range(1):
	ix = i
	iy = (i+1)%2

	axes[i].scatter(chains[ix, :], chains[iy, :], s=1)
	axes[i].set_xlabel(labels[ix])	
	axes[i].set_ylabel(labels[iy])

axes[2].plot(acceleration_measurement)
axes[3].plot(logP_arr)

plt.show()