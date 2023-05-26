import numpy as np 
from scipy import integrate, interpolate
from scipy.linalg import expm

import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator

# internal imports
from .physics import environment
from .physics import launchpad
from .common import frames
from .common import quaternion

class RocketSimulator(object):
	def __init__(
		self, 
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
		body_length, 
		tail_length, 
		lift_rate_coefficient_body, 
		drag_coefficient_body, 
		lift_rate_coefficient_tail, 
		drag_coefficient_tail, 
		moment_coefficent_rate_body,
		moment_coefficent_rate_tail,
		position_aerodynamic_center_body=0.5,
		position_aerodynamic_center_tail=0.15,
		position_center_of_thrust=0,
		position_cog=0.75,
		logger=None,
		):
		if not isinstance(pad, launchpad.LaunchPad): 
			raise ValueError(f"pad must be a {launchpad.LaunchPad} instance.")

		if not isinstance(env, environment.Environment):
		 	raise ValueError(f"pad must be a {environment.Environment} instance.")

		self.logger = logger 

		self.t0 = initial_time
		self.t_end = t_end
		self.launch_time = launch_time
		self.pad = pad
		self.env = env

		self._thrust = None
		self.thrust_duration = None

		self.m = mass
		self.inertia_matrix = inertia_matrix

		# aerodynamics
		self.Sref_body = Sref_body
		self.Sref_tail = Sref_tail

		self.C_L_alpha_body = lift_rate_coefficient_body
		self.C_L_alpha_tail = lift_rate_coefficient_tail
		self.C_D_body = drag_coefficient_body
		self.C_D_tail = drag_coefficient_tail

		self.C_m_alpha_body = moment_coefficent_rate_body
		self.C_m_alpha_tail = moment_coefficent_rate_tail
		self.alpha0 = alpha0
		self.body_length = body_length
		self.tail_length = tail_length

		self.states = None
		self.t = None

		self.r_cog_ca_body = (position_aerodynamic_center_body - position_cog)*np.array([1, 0, 0])
		self.r_cog_ca_tail = (position_aerodynamic_center_tail - position_cog)*np.array([1, 0, 0])
		self.r_cog_ct = (position_center_of_thrust - position_cog)*np.array([1, 0, 0])

		if self.logger is not None:
			self.logger.info(f"RocketSimulation class initialized : ")
			self.logger.info(self.__str__)


	def propagate(self, t_end, n_points=1000):
		if self.logger is not None: self.logger.info("Starting rocket state propagator ")

		n_dim = 13

		# compute initia l state
		x0 = np.zeros(n_dim)
		x0[0:3] = self.pad.get_launch_position_ecef()
		x0[6:10] = quaternion.Quaternion(*self.pad.get_launch_attitude()).get_coefficients()

		if self.logger is not None: self.logger.info(f"Initial step set x0={x0}")

		# State propagation
		states = []
		time_points = []

		def stop_condition(t, x):
			lon, lat, h = frames.ecef_to_geodetic(x[0:3])
			print(h)
			if t > self.launch_time + 10.0:
				

				if h <= 0: 
					print("stop condition triggered")
					return 0

			return 1

		stop_condition.terminal = True

		counter = 0
		ts_mem = 0

		# internal values

		def log_eta(ts):
			nonlocal counter, ts_mem

			if ts > ts_mem:
				progress = (ts-self.t0)/(t_end-self.t0)*100.0

				self.logger.info(f"Propagating states time step t = {ts:.3f} s - progress {progress:.2f} %")

				ts_mem = ts
				counter = 0
			else:
				counter += 1

				if counter == 40:
					ts_mem = ts
					counter = 0

		def state_transition_model_function(t, x):
			if self.logger is not None: 
				log_eta(t)

			# get previous states
			r_ecef 		= x[0:3]
			v_ecef 		= x[3:6]

			q 			= x[6:10]
			omega_ecef 	= x[10:13]

			q = quaternion.Quaternion(*q).normalize()
			omega_body = np.dot(q.DCM().T, frames.ecef_to_enu(omega_ecef, self.pad.get_launch_position_ecef(), only_rotation=True))

			# compute inputs
			wind_vel = self.env.get_wind_velocity(t, r_ecef).reshape((3, ))*0

			# get external forces
			F, M = self.compute_forces_and_moments_body_frame(t, r_ecef, v_ecef, wind_vel, q, omega_body)
			del wind_vel

			# EOM :	
			accel_body_frame = F/self.m
			alpha_body_frame = np.dot(np.linalg.inv(self.inertia_matrix), M)

			#print("accel_body_frame, ", accel_body_frame)
			#print("alpha_body_frame, ", alpha_body_frame)
			del F, M

			# change RF body -> ECEF :
			# print("before, ", accel_body_frame, " / after, ", np.dot(q.DCM(), accel_body_frame))
			accel_ecef_new = frames.enu_to_ecef(np.dot(q.DCM(), accel_body_frame), self.pad.get_launch_position_ecef(), only_rotation=True)
			alpha_ecef_new = frames.enu_to_ecef(np.dot(q.DCM(), alpha_body_frame), self.pad.get_launch_position_ecef(), only_rotation=True)

			# propagate states :
			dx = x.copy()

			dx[0:3] 		= v_ecef
			dx[3:6] 		= accel_ecef_new

			wx, wy, wz = omega_body
			q0, q1, q2, q3 = dx[6:10]

			dx[6] = (-q1*wx - q2*wy - q3*wz)/2
			dx[7] = ( q0*wx - q3*wy + q2*wz)/2
			dx[8] = ( q3*wx + q0*wy - q1*wz)/2
			dx[9] = (-q2*wx + q1*wy + q0*wz)/2

			dx[10:13] 		= alpha_ecef_new

			del r_ecef, v_ecef, accel_ecef_new, omega_ecef, q, alpha_ecef_new

			return dx

		if self.logger is not None: self.logger.info(f"Running propagator over t={(self.t0, t_end)}")
		solution = integrate.solve_ivp(state_transition_model_function, (self.t0, t_end), x0, method="Radau", rtol=1e-5, events=(stop_condition,))

		if self.logger is not None: self.logger.info("Propagation loop terminated. Gathering results...")

		self.t = np.linspace(solution["t"][0], solution["t"][-1], n_points)

		self.states = interpolate.interp1d(solution["t"], solution["y"], kind='quadratic')(self.t)
		self.accel = np.diff(self.states[3:6])/np.diff(self.t)

		self.w = np.array([np.dot(quaternion.Quaternion(*self.states[6:10, i]).DCM().T, frames.ecef_to_enu(self.states[10:13, i], self.states[0:3, i])) for i in range(len(self.t))])
		print(self.w.shape)
		self.w = self.w.T
		print(self.w.shape)
		if self.logger is not None: self.logger.info("End of simulation")

	def compute_forces_and_moments_body_frame(self, t, r_ecef, v_ecef, wind_vel, q, omega_body):
		if self.logger is not None: self.logger.debug(f"computing rocket forces and moments t={t}s")

		v_ecef_tot = v_ecef + wind_vel

		v_bf_body = np.dot(q.DCM().T, frames.ecef_to_enu(v_ecef_tot, r_ecef, only_rotation=True))

		# aerodynamic forces
		F_a_body = self.compute_aerodynamic_force(t, r_ecef, v_bf_body, self.C_L_alpha_body, self.C_D_body, self.Sref_body, omega_body, np.zeros(3))
		F_a_tail = self.compute_aerodynamic_force(t, r_ecef, v_bf_body, self.C_L_alpha_tail, self.C_D_tail, self.Sref_tail, omega_body, self.r_cog_ca_tail)
		
		# print("F_a_body, ", F_a_body)
		# print("F_a_tail, ", F_a_tail)
		
		F_aerodynamic = F_a_body + np.array([4, 2, 2])*F_a_tail

		# aerodynamic moments
		M_F_a_body = np.cross(self.r_cog_ca_body, F_a_body)
		M_F_a_tail = np.cross(self.r_cog_ca_tail, F_a_tail)

		# print("M_F_a_body, ", M_F_a_body)
		# print("M_F_a_tail, ", M_F_a_tail)

		M_a_body = 0*self.compute_aerodynamic_moment(t, r_ecef, v_bf_body, self.C_m_alpha_body, self.Sref_body, self.body_length, omega_body, np.zeros(3))
		M_a_tail = 0*self.compute_aerodynamic_moment(t, r_ecef, v_bf_body, self.C_m_alpha_tail, self.Sref_tail, self.tail_length, omega_body, self.r_cog_ca_tail)

		# print("M_a_body, ", M_a_body)
		# print("M_a_tail, ", M_a_tail)

		M_aerodynamic = M_F_a_body + M_a_body + 2*(M_F_a_tail + M_a_tail)
		# print("M_aerodynamic, ", M_a_tail)

		# Weight 
		g_enu = frames.ecef_to_enu( self.env.g(r_ecef), r_ecef, only_rotation=True)
		F_weight = np.dot(q.DCM().T, g_enu) * self.m
		# print("F_weight, ", F_weight)

		# THRUST 
		# Force
		F_thrust = self.thrust(t - self.launch_time, r_ecef)
		# print("F_thrust, ", F_thrust)

		# Moment
		M_thrust = np.cross(self.r_cog_ct, F_thrust)
		# print("M_thrust, ", M_thrust)

		# Pad force 
		F_ext = F_aerodynamic + F_weight + F_thrust
		F_pad = self.pad.get_forces(F_ext, r_ecef, t, self.launch_time)
		# print("F_ext, ", F_ext)
		# print("F_pad, ", F_pad)

		# Total forces and Moments
		F_ext = F_ext + F_pad
		M_ext = M_aerodynamic + M_thrust

		# print("F_ext, ", F_ext)
		# print("M_ext, ", M_ext)

		if self.logger is not None: self.logger.debug(f"ECEF : F={F_ext}, M={M_ext}")

		return F_ext, M_ext


	def set_motor_properties(self, thrust, time, thrust_direction):
		if not isinstance(thrust, np.ndarray): 
			raise ValueError("thrust is not an np.dnarray")

		if not isinstance(time, np.ndarray): 
			raise ValueError("time is not an np.dnarray")

		if not (np.size(time) == np.size(thrust)): 
			raise ValueError("time and thrust need to be the same size")

		self._thrust = interpolate.interp1d(time, thrust, kind='linear')
		self.thrust_duration = time[-1] - time[0]
		self.thrust_direction = thrust_direction

	def thrust(self, t, r):
		if self._thrust is None:
			raise "Error : please initialize motor properties before attempting to get thrust"
		else:
			if t >= 0.0 and t <= self.thrust_duration:
				thust = self._thrust(t)
			else:
				thust = 0.0

		thrust_dir = self.thrust_direction.reshape((3,))
		thust = thust*thrust_dir

		if self.logger is not None: self.logger.debug(f"rocket thrust : {thust}")

		return thust

	def get_position(self, t):
		if self.states is None: 
			raise ValueError("States is None, please run propagator before trying to compute states")

		return self.states(t)[0:3,:]

	def get_velocity(self, t):
		if self.states is None: 
			raise ValueError("States is None, please run propagator before trying to compute states")

		return self.states(t)[3:6,:]

	def get_acceleration(self, t):
		if self.states is None: 
			raise ValueError("States is None, please run propagator before trying to compute states")

		return self.states(t)[6:9,:]

	def get_attitude(self, t):
		if self.states is None: 
			raise ValueError("States is None, please run propagator before trying to compute states")

		return self.states(t)[9:12,:]

	def get_angular_velocity(self, t):
		if self.states is None: 
			raise ValueError("States is None, please run propagator before trying to compute states")

		return self.states(t)[12:15,:]


	def compute_aerodynamic_force(self, t, r_ecef, vel, C_L_alpha, C_D, S_ref, omega, r_pos):
		vel_apparent = vel + np.cross(omega, r_pos)

		vel_apparent_norm = np.linalg.norm(vel_apparent)
		vel_norm = np.linalg.norm(vel)

		if self.logger is not None: 
			self.logger.debug(f"aerodynamic force : velocity={-vel} / effective_velocity={-vel_apparent}")

		# compute angle of attack and side angle
		if vel_apparent_norm <= 0.0001:
			F_aero = np.zeros(3)
		else:
			lon, lat, z = frames.ecef_to_geodetic(r_ecef)
			rho = self.env.atmospheric_model.rho(z, t)

			alpha = np.arcsin(vel[2]/vel_norm)
			alpha_apparent = np.arcsin(vel_apparent[2]/vel_apparent_norm)

			beta = np.arctan2(vel[1], vel[0])
			beta_apparent = np.arctan2(vel_apparent[1], vel_apparent[0])

			pdyn = 0.5 * rho * vel_apparent_norm**2

			FLa = pdyn * S_ref * C_L_alpha*(alpha_apparent - self.alpha0)
			FLb = pdyn * S_ref * C_L_alpha*(beta_apparent - self.alpha0)
			FD = pdyn * S_ref * C_D

			F_aero = -np.array([FD, FLb, FLa])
			F_aero = frames.aero_to_body_frame(F_aero, alpha, beta)

		if self.logger is not None: self.logger.debug(f"aerodynamic force : Body reference frame -> F_aero= {F_aero}")

		return F_aero

	def compute_aerodynamic_moment(self, t, r_ecef, vel, C_m_alpha, S_ref, L, omega, r_pos):
		# compute angle of attack and side angle		
		vel_apparent = vel + np.cross(omega, r_pos)

		vel_apparent_norm = np.linalg.norm(vel_apparent)
		vel_norm = np.linalg.norm(vel)

		if self.logger is not None: self.logger.debug(f"aerodynamic force : velocity={-vel} / effective_velocity={-vel_apparent}")

		if vel_norm <= 0.0001:
			M_aero = np.zeros(3)
		else:
			lon, lat, z = frames.ecef_to_geodetic(r_ecef)
			rho = self.env.atmospheric_model.rho(z, t)

			alpha = np.arcsin(vel[2]/vel_norm)
			beta = np.arctan2(vel[1], vel[0])

			pdyn = 0.5*rho*vel_norm**2
			M_alpha = pdyn * S_ref * L * C_m_alpha*(alpha - self.alpha0)
			M_beta 	= pdyn * S_ref * L * C_m_alpha*(beta - self.alpha0)


			M_aero = np.array([0, M_alpha, M_beta])*0
			# TODO vérifier signe des moments
			M_aero = frames.aero_to_body_frame(M_aero, alpha, beta)

		if self.logger is not None: self.logger.debug(f"aerodynamic moments : Body reference frame -> M_aero= {M_aero}")

		return M_aero

	def plot_states(self):
		vars_names = [
			["$x(t)$ $[m]$", "$y(t)$ $[m]$", "$z(t)$ $[m]$"], 
			["$v_{x}(t)$ $[m/s]$", "$v_{y}(t)$ $[m/s]$", "$v_{z}(t)$ $[m/s]$"], 
			["$q_{0}(t)$ $[-]$", "$q1_{1}(t)$ $[-]$", "$q_{2}(t)$ $[-]$", "$q_{3}(t)$ $[-]$"], 
			["$\\omega_{x}(t)$ $[rad.s^{-1}]$", "$\\omega_{y}(t)$ $[rad.s^{-1}]$", "$\\omega_{z}(t)$ $[rad.s^{-1}]$"]]

		titles = ["position", "velocity", "attitude quaternion", "angular velocity"]

		if self.logger is not None: self.logger.info(f"plotting rocket propagated states : {vars_names}")

		index = 0
		for vec_id, title in enumerate(titles):
			if self.logger is not None: self.logger.debug(f"plotting {title}...")

			fig = plt.figure()
			fig.suptitle(title, fontsize=16)

			N = len(vars_names[vec_id])
			Nplots = N

			if title == "attitude quaternion":
				Nplots = N + 1

			axes = fig.subplots(Nplots, 1, sharex=True)

			for i in range(N):
				axes[i].plot(self.t, self.states[index,:], "-b")

				axes[i] = self.__set_plot_style(axes[i])
				axes[i].set_xlabel("t")			
				axes[i].set_ylabel(vars_names[vec_id][i])

				axes[i].grid()

				axes[i].set_xlim([self.t[0], self.t[-1]])

				if i < len(axes)-1:
					plt.setp(axes[i].get_xticklabels(), visible=False)

				index += 1

		fig.align_ylabels(axes)
		
		# plotting aerodynamic parameters
		titles = ["$V_n(t)$ [m/s]", "$\\alpha(t)$ [deg]", "$\\beta(t)$ [deg]"]
		if self.logger is not None: self.logger.info(f"plotting {titles}...")

		fig = plt.figure()
		axes = fig.subplots(3, 1, sharex=True)

		vel = self.states[3:6, :]
		q = quaternion.Quaternion(*self.states[6:10, :]).normalize()	

		vel_body_frame = np.einsum("ijk,ki->ji", q.DCM().T, vel)
		V = np.linalg.norm(vel_body_frame[:,:], axis=0)
		alpha = np.arcsin(vel_body_frame[2,:]/V)
		beta = np.arcsin(vel_body_frame[1,:]/np.linalg.norm(vel_body_frame[0:2,:], axis=0))

		axes[0].plot(self.t, V, "-b")
		axes[1].plot(self.t, alpha, "-b")
		axes[2].plot(self.t, beta, "-b")

		for i in range(len(titles)):
			axes[i] = self.__set_plot_style(axes[i])

			axes[i].set_xlabel("$t$ [s]")			
			axes[i].set_ylabel(titles[i])
			axes[i].grid()

			if i < len(axes)-1:
				plt.setp(axes[i].get_xticklabels(), visible=False)

		fig.align_ylabels(axes)

		# plotting acceleration
		titles = ["$\\gamma_{x}(t)$ $[m.s^{-2}]$", "$\\gamma_{y}(t)$ $[m.s^{-2}]$", "$\\gamma_{z}(t)$ $[m.s^{-2}]$"]
		if self.logger is not None: self.logger.info(f"plotting {titles}...")

		fig = plt.figure()
		fig.suptitle("Rocket acceleration", fontsize=16)

		axes = fig.subplots(3, 1, sharex=True)

		for i in range(len(titles)):
			axes[i].plot(self.t[:-1], self.accel[i], "-b")

			axes[i] = self.__set_plot_style(axes[i])

			axes[i].set_xlabel("$t$ [s]")			
			axes[i].set_ylabel(titles[i])
			axes[i].grid()

			if i < len(axes)-1:
				plt.setp(axes[i].get_xticklabels(), visible=False)

		fig.align_ylabels(axes)

		if self.logger is not None: self.logger.info(f"plotting 3d trajectory...")
		r = self.states[0:3]-self.pad.get_launch_position_ecef()[:,None]

		fig = plt.figure()
		fig.suptitle(title, fontsize=16)

		ax = fig.add_subplot(111, projection='3d')
		ax.plot(r[0], r[1], r[2], "-r")

		ax.set_xlabel(r'$x [m]$')
		ax.set_ylabel(r'$y [m]$')
		ax.set_zlabel(r'$z [m]$')

		plt.show()

	def __str__(self):
		print("Rocket Simulation class : ")

		print("t0 : ", self.t0)
		print("t_end : ", self.t_end)
		print("launch_time : ", self.launch_time)
		print("pad : ", self.pad)
		print("env : ", self.env)

		print("thrust : ", self._thrust)
		print("thrust_duration : ", self.thrust_duration)

		print("m : ", self.m)
		print("inertia_matrix : ", self.inertia_matrix)

		print("Sref_body : ", self.Sref_body)
		print("Sref_tail : ", self.Sref_tail)
		print("C_L_alpha_body : ", self.C_L_alpha_body)
		print("C_L_alpha_tail : ", self.C_L_alpha_tail)
		print("C_D_body : ", self.C_D_body)
		print("C_D_tail : ", self.C_D_tail)

		print("C_m_alpha_body : ", self.C_m_alpha_body)
		print("C_m_alpha_tail : ", self.C_m_alpha_tail)
		print("alpha0 : ", self.alpha0)
		print("body_length : ", self.body_length)
		print("tail_length : ", self.tail_length)

		print("r_cog_ca_body : ", self.r_cog_ca_body)
		print("r_cog_ca_tail : ", self.r_cog_ca_tail)
		print("r_cog_ct : ", self.r_cog_ct)

		return ""

	def __set_plot_style(self, ax):
		ax.tick_params(which="minor", direction="in", bottom=True, top=True, left=True, right=True)
		ax.tick_params(which="major", direction="in", bottom=True, top=True, left=True, right=True)

		ax.xaxis.set_minor_locator(AutoMinorLocator())
		ax.yaxis.set_minor_locator(AutoMinorLocator())

		plt.subplots_adjust(wspace=0, hspace=0)

		return ax