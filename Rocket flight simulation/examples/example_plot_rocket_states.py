import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from rocket_simulator.common import frames
from rocket_simulator.common import quaternion

def set_plot_style(ax):
	ax.tick_params(which="minor", direction="in", bottom=True, top=True, left=True, right=True)
	ax.tick_params(which="major", direction="in", bottom=True, top=True, left=True, right=True)

	ax.xaxis.set_minor_locator(AutoMinorLocator())
	ax.yaxis.set_minor_locator(AutoMinorLocator())

	plt.subplots_adjust(wspace=0, hspace=0.2)

	ax.tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)

	ax.grid()

	return ax

# -------------------------------------
# 	   setup simulation parameters
# -------------------------------------

# get rocket states
states 	= np.load("sim_rocket_states.npy")[:,::10]
t 		= np.load("sim_rocket_t.npy")[::10]
w 		= np.load("sim_rocket_w.npy")[:,::10]

vars_names = [
		["$x(t)$ \n [$m$]", "$y(t)$ \n [$m$]", "$z(t)$ \n [$m$]"], 
		["$v_{x}(t)$ \n [$m/s$]", "$v_{y}(t)$ \n [$m/s$]", "$v_{z}(t)$ \n [$m/s$]"], 
		["$q_{0}(t)$ \n [$-$]", "$q1_{1}(t)$ \n [$-$]", "$q_{2}(t)$ \n [$-$]", "$q_{3}(t)$ \n [$-$]"],
		["$\\omega_{x}(t)$ \n [$rad.s^{-1}$]", "$\\omega_{y}(t)$ \n [$rad.s^{-1}$]", "$\\omega_{z}(t)$ \n [$rad.s^{-1}$]"],
	]

titles = ["position", "velocity", "attitude quaternion", "angular velocity"]

index = 0
for vec_id, title in enumerate(titles):
	fig = plt.figure()
	fig.suptitle(title, fontsize=16)

	N = len(vars_names[vec_id])
	axes = fig.subplots(N, 1, sharex=False)

	for i in range(N):
		set_plot_style(axes[i])
		axes[i].plot(t, states[index], "-b")
		index += 1

		axes[i].set_ylabel(vars_names[vec_id][i])

	fig.align_labels()
titles = ["position enu", "velocity enu", "angular velocity enu"]

index = 0
for vec_id, title in enumerate(titles):
	print("plotting ", title)
	fig = plt.figure()
	fig.suptitle(title, fontsize=16)

	if vec_id == 2:
		index += 4
		vec_id += 1

	print(vec_id)

	N = len(vars_names[vec_id])
	axes = fig.subplots(N, 1, sharex=False)

	if vec_id == 0:
		data_enu = np.asfarray([frames.ecef_to_enu(states[index:index+3, i] - states[0:3, 0], states[0:3, 0], only_rotation=True) for i in range(len(t))])
	else:
		data_enu = np.asfarray([np.dot(quaternion.Quaternion(*states[6:10, i]).DCM().T, frames.ecef_to_enu(states[index:index+3, i], states[0:3, 0], only_rotation=True)) for i in range(len(t))])
	print(data_enu)
	print(N)
	
	for i in range(N):
		set_plot_style(axes[i])
		axes[i].plot(t, data_enu[:, i], "-b")

		axes[i].set_ylabel(vars_names[vec_id][i])
	
	fig.align_labels()

	index += 3

plt.show()
