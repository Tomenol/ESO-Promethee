import numpy as np
from scipy import interpolate

from . import base
from ...common import frames

"""
Implements a model of the wind velocity and direction.
The wind velocity magnitude and azimuth are computed by interpolating predifined tables (as a function of altitude).
"""

class WindModelTables(base.WindModelBase):
	"""
	Implementation of a wind model computing the velocity vector as a function of altitude from predifined
	velocity magnitude and azimuth tables.

	Note that the implementation of this class does only allows for the simulation of static winds.
	"""
	def __init__(self, epoch, altitude, wind_magnitude, azimuth, deg=False, logger=None):
		"""
		Implementation of a wind model computing the velocity vector as a function of altitude from predifined
		velocity magnitude and azimuth tables.
		Note that the implementation of this class does only allows for the simulation of static winds.

		Parameters :
		------------
			epoch : float
				starting simulation date in years.

			altitude : np.ndarray(N,)
				Altitude points (in meter) at which the wind magnitude and azimuth values will be defined
		
			wind_magnitude : np.ndarray(N,)
				Wind magnitude (meters per second) value at the specified altitudes

			azimuth : np.ndarray(N,)
				Wind azimuth value at the specified altitudes
		
			deg : bool (optional)
				If True, all the angles are in degrees. If False, (default value) all the angles are in radians.

			logger : logging
				logging class used to log computation status 

		Examples :
		----------
			One can define a simple wind model as follow :
	
			first, we need to import the main toolbox
			>>> import simulator
	
			Then, one mst define the wind magnitude and azimuth tables :
			>>> altitude 		= np.array([0,  50, 100, 150, 200, 250, 300, 5000]) # m
			>>> wind_magnitude 	= np.array([5,  3,  5,   10,  12,  15,  15,  30]) 	# m/s
			>>> azimuth 		= np.array([0,  0 , 10,  0,   0,   10,  30,  20])	# deg

			Then, the model can be instantiated :
			>>> wind_model = simulator.WindModelTables(altitude, wind_magnitude, azimuth, deg=True)

			And then the velocity can be retrieved by calling :
			>>> t = 10 # seconds
			>>> r = np.array([6378e3, 100, 100])
			>>> wind_velocity = wind_model.velocity(r, t)

		"""	
		super().__init__(epoch, logger=logger)

		self.vel = interpolate.interp1d(altitude, wind_magnitude, bounds_error=False, fill_value=(wind_magnitude[0], wind_magnitude[-1]))

		# convert to radians
		if deg is True:
			azimuth = azimuth*np.pi/180.0 

		if isinstance(azimuth, np.ndarray):
			if np.size(azimuth) != np.size(altitude):
				raise ValueError("azimuth must have the same size as the altitude array.")
			else:
				self.az = interpolate.interp1d(altitude, azimuth, bounds_error=False, fill_value=(azimuth[0], azimuth[-1]))
		else:
			if not isinstance(azimuth, int) and not isinstance(azimuth, float):
				raise ValueError("azimuth must be an array, int or float.")

			self.az = azimuth

		if self.logger is not None:
			self.logger.info("Custom tabulated wind model initialized :")
			self.logger.info("altitude : ", altitude)
			self.logger.info("wind_magnitude : ", wind_magnitude)
			self.logger.info("azimuth : ", azimuth)


	def velocity(self, r, t):
		"""
		Computes the velocity wind at the given position and time by performing a linear interpolation of the wind magnitude and azimuth in
		predifined tables.

		Parameters :
		------------
		r : np.ndarray (3,)
			ECEF position at which the wind velocity is to be computed.

		t : float
			time in seconds at which the wind velocity is to be computed.

		Returns :
		---------
			np.ndarray : (3,)
				wind velocity vector in the ECEF frame.

		"""
		# get geodetic coordinates to get the altitude of the point over the reference geoid
		lon, lat, h = frames.ecef_to_geodetic(r)

		# get velocity direction and azimuth
		velocity_norm = self.vel(h)
		dir_angle = self.direction(r, t, deg=False)

		# compute the wind velocity in the ENU frame and then in the ECEF frame 
		vel_unit_vector_enu = np.array([np.sin(dir_angle), np.cos(dir_angle), 0])
		velocity_vector_ecef = velocity_norm * frames.enu_to_ecef(vel_unit_vector_enu, r)

		return velocity_vector_ecef



	def direction(self, r, t, deg=False):
		"""
		Computes the wind direction at the given position and time by performing a linear interpolation of the wind azimuth in
		predifined tables.

		Parameters :
		------------
		r : np.ndarray (3,)
			ECEF position at which the wind velocity is to be computed.

		t : float
			time in seconds at which the wind velocity is to be computed.
		
		deg : bool (optional)
			if True, the direction will be returned in degrees

		Returns :
		---------
			float :
				wind direction (azimuth).

		"""
		if not isinstance(self.az, int) and not isinstance(self.az, float):
			lon, lat, h = frames.ecef_to_geodetic(r)
			dir_angle = self.az(h)
		else:
			dir_angle = self.az

		if deg is True:
			dir_angle = dir_angle*180.0/np.pi

		return dir_angle


	def __str__(self):
		"""
		Prints class data and informations
		"""
		print("{Tabulated Wind Model}")
		print("")