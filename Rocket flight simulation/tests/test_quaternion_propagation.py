import numpy as np
import matplotlib.pyplot as plt

import rocket_simulator
from rocket_simulator.quaternion import Quaternion 

initial_time = 0
launch_time = 0
t_end = 0.03
dt = 0.01
npoints = int(t_end/dt)

initial_rpy = np.array([0, -90, 0])

env = rocket_simulator.Environment()
pad = rocket_simulator.LaunchPad(45, 45, 0, env, initial_attitude_euler=initial_rpy, deg=True)

e1 = np.eye(3)
e2 = np.dot(Quaternion(*pad.get_launch_attitude(deg=False), deg=False).DCM(), e1) 

fig	= plt.figure()
ax = fig.add_subplot(111, projection="3d")

for i in range(3):
	ax.plot([0, e1[0,i]], [0, e1[1,i]], [0, e1[2,i]], "-r")
for i in range(3):
	ax.plot([0, e2[0,i]], [0, e2[1,i]], [0, e2[2,i]], "-b")


ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

V_rocket = np.array([0.5, 0, 0])
V_rocket_ecef = np.dot(Quaternion(*pad.get_launch_attitude(deg=False), deg=False).DCM(), V_rocket) 

ax.plot([0, V_rocket_ecef[0]], [0, V_rocket_ecef[1]], [0, V_rocket_ecef[2]], "-m")

print(e1)
print(e2)

plt.show()


# n_points = 1000
# dt = 0.001
# omega = np.array([0.1, 0.1, -0.1])

# euler_angles = np.empty((3, n_points))
# t = np.arange(0, n_points*dt, dt)

# for i in range(n_points):
# 	q = q.propagate(omega, dt, body_frame=True)

# 	euler_angles[:, i] = q.convert_to_attitude_vector(deg=True)

# fig = plt.figure()
# axes = fig.subplots(3, 1)

# labels = [r"$\Phi(t)$", r"$\theta(t)$", r"$\Psi(t)$"]

# for i, ax in enumerate(axes):
# 	ax.plot(t, euler_angles[i])

# 	ax.set_ylabel(labels[i] + " [deg]")	
# 	ax.set_xlabel("t [s]")

# 	ax.grid()
# 	ax.set_xlim([t[0], t[-1]])

# fig.suptitle("Euler angles (quaternion propagation)")
# plt.show()
