import numpy as np 
import matplotlib.pyplot as plt

data_np_raw = np.load("flight_data.npy")

# plot raw accel
t = data_np_raw["t"]

accel1 = np.array([data_np_raw["ax1"], data_np_raw["ay1"], data_np_raw["az1"]]).astype(float)
accel2 = np.array([data_np_raw["ax2"], data_np_raw["ay2"], data_np_raw["az2"]]).astype(float)

fig = plt.figure()
ax = fig.subplots(3, 1, sharex=True) 
labels_accel = ["$\\gamma_x(t)$ \n [$m/s^2$]", "$\\gamma_y(t)$ \n [$m/s^2$]", "$\\gamma_z(t)$ \n [$m/s^2$]"]

for i in range(3):
	ax[i].plot(t, accel1[i, :]*9.81, "-b")
	ax[i].grid()
	ax[i].tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)
for i in range(3):
	ax[i].plot(t, accel2[i, :]*9.81, "-r")
fig.legend(["Accelerometer 1", "Accelerometer 2"], loc="lower center")
fig.align_ylabels()

ax[-1].set_xlabel("$t$ [$s$]")
for i in range(3):
	ax[i].set_ylabel(labels_accel[i])


# compute mean acceleration at 90s (no need to include g because the rocket is lying on the side)
i_start_static = np.argmax(data_np_raw["t"] >= 900000.0)

a_mean_1 = np.mean(accel1[:, i_start_static:i_start_static+30], axis=1)
a_mean_2 = np.mean(accel2[:, i_start_static:i_start_static+30], axis=1)

a_std_1 = np.sqrt(np.var(accel1[:, i_start_static:i_start_static+30], axis=1))
a_std_2 = np.sqrt(np.var(accel2[:, i_start_static:i_start_static+30], axis=1))

b_accel_1 = np.array([0, 0, a_mean_1[2]])
b_accel_2 = np.array([0, 0, a_mean_2[2]])

print(a_mean_1)
print(a_std_1)
print(a_mean_2)
print(a_std_2)

print(np.linalg.norm(a_mean_1))
print(np.linalg.norm(a_mean_2))



data_msk = np.logical_and(data_np_raw["t"]/1000.0 >= 4250.0, data_np_raw["t"]/1000.0 <= 4510.0)
data_np = data_np_raw[data_msk]

data_np["t"] = (data_np["t"] - data_np["t"][0])/1000.0
data_np["t_slave"] = (data_np["t_slave"] - data_np["t"][0])/1000.0

t = data_np["t"]

accel1 = np.array([data_np["ax1"], data_np["ay1"], data_np["az1"]]).astype(float)
accel2 = np.array([data_np["ax2"], data_np["ay2"], data_np["az2"]]).astype(float)

# compute mean acceleration during start
a_mean_1 = np.mean(accel1[:, 0:15], axis=1)
a_mean_2 = np.mean(accel2[:, 0:15], axis=1)

a_std_1 = np.sqrt(np.var(accel1[:, 0:15], axis=1))
a_std_2 = np.sqrt(np.var(accel2[:, 0:15], axis=1))

print(a_mean_1)
print(a_std_1)
print(a_mean_2)
print(a_std_2)

print(np.linalg.norm(a_mean_1))
print(np.linalg.norm(a_mean_2))


# compute initial attitude
accel_init = ((a_mean_1 - b_accel_1)*a_std_2**2 + a_std_1**2*(a_mean_2 - b_accel_2))/(a_std_2**2 + a_std_1**2)

initial_elevation = np.arctan(np.linalg.norm(accel_init[0:2])/accel_init[2])

print("initial_elevation : ", initial_elevation*180.0/np.pi)


accel = ((accel1 - b_accel_1[:, None])*a_std_2[:, None]**2 + a_std_1[:, None]**2*(accel2 - b_accel_2[:, None]))/(a_std_2[:, None]**2 + a_std_1[:, None]**2)
accel_z_enu = accel[2]*np.cos(initial_elevation)


fig = plt.figure()
ax = fig.subplots(3, 1, sharex=True)


for i in range(3):
	ax[i].plot(t, accel1[i, :]*9.81, "-b")
	ax[i].grid()
	ax[i].tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)
for i in range(3):
	ax[i].plot(t, accel2[i, :]*9.81, "-r")
for i in range(3):
	ax[i].plot(t, accel[i]*9.81, "-k", alpha=0.8)
fig.legend(["Accelerometer 1", "Accelerometer 2", "Processed"], loc='lower center')
ax[-1].set_xlabel("$t$ [$s$]")
for i in range(3):
	ax[i].set_ylabel(labels_accel[i])





# compute altitude during ascention
i_start = np.argmax(accel_z_enu>1.0)-2
i_end = np.argmax(np.logical_and(accel_z_enu>0.0, t>18.5))-1

print(accel_z_enu[i_start])
print(i_start)
print(i_end)



# compute mean acceleration during end
a_mean_1 = np.mean(accel1[:, -20:], axis=1)
a_mean_2 = np.mean(accel2[:, -20:], axis=1)

a_std_1 = np.sqrt(np.var(accel1[:, -20:], axis=1))
a_std_2 = np.sqrt(np.var(accel2[:, -20:], axis=1))

a_std = np.sqrt((2*a_std_1**2*a_std_2**2)/(a_std_2**2 + a_std_1**2))

print(a_mean_1)
print(a_std_1)
print(a_mean_2)
print(a_std_2)

print(np.linalg.norm(a_mean_1))
print(np.linalg.norm(a_mean_2))

# plot angular velocity 
omega1 = np.array([data_np["wx1"], data_np["wy1"], data_np["wz1"]]).astype(float)
omega2 = np.array([data_np["wx2"], data_np["wy2"], data_np["wz2"]]).astype(float)

fig = plt.figure()
ax = fig.subplots(3, 1, sharex=True)


# get biases
w_mean_1 = np.mean(omega1[:, 0:10], axis=1)
w_mean_2 = np.mean(omega2[:, 0:10], axis=1)

w_std_1 = np.sqrt(np.var(omega1[:, 0:10], axis=1))
w_std_2 = np.sqrt(np.var(omega2[:, 0:10], axis=1))

w_std = np.sqrt((2*w_std_1**2*w_std_2**2)/(w_std_2**2 + w_std_1**2))

b_omega_1 = w_mean_1
b_omega_2 = w_mean_2

omega = ((omega1 - b_omega_1[:, None])*w_std_2[:, None]**2 + w_std_1[:, None]**2*(omega2 - b_omega_2[:, None]))/(w_std_2[:, None]**2 + w_std_1[:, None]**2)
omega_xy_enu = np.linalg.norm(omega[0:2], axis=0)

labels_omega = ["$\\omega_x(t)$ \n [$deg/s$]", "$\\omega_y(t)$ \n [$deg/s$]", "$\\omega_z(t)$ \n [$deg/s$]"]

for i in range(3):
	ax[i].plot(t, omega1[i, :], "-b")
	ax[i].grid()
	ax[i].tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)
for i in range(3):
	ax[i].plot(t, omega2[i, :], "-r")
for i in range(3):
	ax[i].plot(t, omega[i], "-k", alpha=0.8)
ax[-1].set_xlabel("$t$ [$s$]")
fig.legend(["Gyrometer 1", "Gyrometer 2", "Processed"], loc='lower center')
for i in range(3):
	ax[i].set_ylabel(labels_omega[i])




theta = np.zeros(len(t[i_start:i_end]))
theta[0] = initial_elevation*180.0/np.pi

for i in range(i_start+1, i_end):
	dt   = t[i] - t[i-1]
	theta[i-i_start] = theta[i-i_start-1] + 0.5*dt*(omega_xy_enu[i] + omega_xy_enu[i-1])

fig = plt.figure()
ax = fig.add_subplot(211)
ax1 = fig.add_subplot(212)


t = t - t[i_start]

ax.plot(t[i_start:i_end], omega_xy_enu[i_start:i_end], "-b")
ax1.plot(t[i_start:i_end], theta, "-b")

ax.grid()
ax1.grid()
ax1.set_xlabel("$t$ [$s$]")

z = np.zeros(len(t[i_start:i_end]))
v = np.zeros(len(t[i_start:i_end]))

theta = theta*np.pi/180

for i in range(i_start+1, i_end):
	dt   = t[i] - t[i-1]
	v[i-i_start] = v[i-i_start-1] + 0.5*dt*(accel[2,i]*np.cos(theta[i-i_start]) + accel[2,i-1]*np.cos(theta[i-i_start-1]) - 2)*9.81
	z[i-i_start] = z[i-i_start-1] + 0.5*dt*(v[i-i_start] + v[i-i_start-1])



# pressure
# plot angular velocity 
pressure = data_np["P"].astype(float)
temperature = data_np["T"].astype(float) + 273.15

fig = plt.figure()
ax = fig.subplots(2, 1, sharex=True)

ax[0].plot(t, pressure, "-b")
ax[1].plot(t, temperature, "-b")

ax[0].set_ylabel("$P$ [$Pa$]")
ax[1].set_ylabel("$T$ [$K$]")

ax[0].grid()
ax[1].grid()
ax[-1].set_xlabel("$t$ [$s$]")
for i in range(2):
	ax[i].tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)

# get biases
P_mean = np.mean(pressure[0:10], axis=0)
T_mean = np.mean(temperature[0:10], axis=0)

P_std = np.sqrt(np.var(pressure[0:10], axis=0))
T_std = np.sqrt(np.var(temperature[0:10], axis=0))

print("P_mean, ", P_mean)
print("T_mean, ", T_mean)

# ISA model
z_init = 436.0
T_z_isa = -0.0065 # K/m
P_0 = 101325
T_0 = 288.15

T_init_isa_th = T_0 + T_z_isa*z_init
P_init_isa_th = P_0 * (1 + z_init*T_z_isa/T_0)**(-9.80665/(287.05*T_z_isa))

b_P = P_mean - P_init_isa_th
b_T = T_mean - T_init_isa_th

z_pressure = -T_0/T_z_isa*((P_0/(pressure[i_start:i_end] - b_P))**(-(287.05*T_z_isa)/(9.80665)) - 1) - z_init

print("z_pressure, ", T_init_isa_th)
print("P_init_isa_th, ", P_init_isa_th)



# Kalman
x = np.zeros((3, i_end-i_start), dtype=float)
x[:,0] = np.array([7.95*np.pi/180, 0, 0, ])


P = np.eye(3, dtype=float)*1e-2

accel = accel*9.80665
omega_xy_enu = omega_xy_enu*np.pi/180

for i in range(1, i_end-i_start):
	dt = t[i+i_start] - t[i+i_start-1]
	print(dt)

	# compute covariance matrices
	Q = np.diag([np.sum(w_std[0:2]**2)*dt, a_std[2]**2*dt**2/2, a_std[2]**2*dt])
	R = P_std**2

	# compute state vector prediction
	x_prediction = x[:, i-1].copy()
	x_prediction[0] = x[0, i-1] + (omega_xy_enu[i_start:i_end][i-1])*dt
	x_prediction[1] = x[1, i-1] + x[2, i-1]*dt
	x_prediction[2] = x[2, i-1] + accel[:,i_start:i_end][2,i-1]*np.cos(x[0, i-1])*dt - 9.90665
	print(omega_xy_enu[i_start:i_end][i-1])


	# compute jacobian (x)
	F = np.eye(3, dtype=float)

	# z_i = x[0, i-1] + (omega_xy_enu[i_start:i_end][i-1] - x[3, i-1])*dt

	# x_prediction[1] = x[1, i-1] + x[2, i-1]*dt
	F[1, 2] = dt

	# x_prediction[2] = x[2, i-1] + (accel[i_start:i_end][2,i-1] - x[4, i-1])*np.cos(x[0, i-1])*dt
	F[2, 0] = -accel[:,i_start:i_end][2,i-1]*np.sin(x[0, i-1])*dt*9.80665

	# compute measurements
	z_pred = P_0 * (1 + (x_prediction[1] + z_init)*T_z_isa/T_0)**(-9.80665/(287.05*T_z_isa)) + b_P

	print(np.array([pressure[i_start:i_end][i], temperature[i_start:i_end][i]]))
	print(z_pred)
	print(x_prediction)

	# Measurements jacobian
	H = np.zeros((1, 3), dtype=float)

	# z_pred[0] = P_0 * (1 + (x_prediction[1] - z_init)*T_z_isa/T_0)**(-9.80665*0.0289644/(8.3144598*T_z_isa))
	H[0, 1] = -9.80665*P_0/(287.04*T_0)*(1 + T_z_isa/T_0*(x_prediction[1] + z_init))**(-9.80665/(287.05*T_z_isa) - 1)

	print(F)
	print(H)


	# Kalman loop
	P_prediction = np.dot(F, np.dot(P, F.T)) + Q

	print("P_prediction, ", P_prediction)

	y = np.array(pressure[i_start:i_end][i] - z_pred)
	S = H.dot(P_prediction.dot(H.T)) + [R]

	K = P_prediction.dot(H.T.dot(np.linalg.inv(S)))
	print(K.dot(y))
	x[:, i] = x_prediction + (K.dot(y)).T
	P = (np.eye(3) - K.dot(H)).dot(P_prediction)

	# normalize quaternion
	P = (P + P.T)/2.0
	print(x[:, i])
		

fig = plt.figure()
ax = fig.add_subplot(311)
ax1 = fig.add_subplot(312)
ax2 = fig.add_subplot(313)


ax.plot(t[i_start:i_end], (accel_z_enu[i_start:i_end]-1.0)*9.90665, "-b")
ax.set_ylabel("$\\gamma_z(t)$ [$m/s^2$]")
ax.legend(["Accelerometer"])
ax.grid()

ax1.plot(t[i_start:i_end], v, "-b")
ax1.plot(t[i_start:i_end][:-1], np.diff(z_pressure)/np.diff(t[i_start:i_end]), "-r")
ax1.plot(t[i_start:i_end], x[2], "-k")

ax1.set_ylabel("$v_z(t)$ [$m/s$]")
ax1.legend(["Accelerometer", "Pressure", "EKF"])
ax1.grid()

ax2.plot(t[i_start:i_end], z, "-b")
ax2.plot(t[i_start:i_end], z_pressure, "-r")
ax2.plot(t[i_start:i_end], x[1], "-k")

ax2.legend(["Accelerometer", "Pressure", "EKF"])
ax2.set_ylabel("$z(t)$ [$m$]")
ax2.set_xlabel("$t$ [$s$]")
ax2.grid()

ax.tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)
ax1.tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)
ax2.tick_params(axis='both', which='both', labelbottom=True, labeltop=False, labelright=True, labelleft=True)
    
plt.show()


