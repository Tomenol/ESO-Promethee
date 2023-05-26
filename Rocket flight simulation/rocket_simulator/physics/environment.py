from abc import ABC, abstractmethod

import numpy as np 
from scipy import interpolate

from . import earth
from ..common import frames

from . import gravity
from . import wind
from . import atmosphere
from . import geomagnetic

class Environment(object):
	"""
	Simulates the rocket environment (wind, atmosphere, ...)
	"""
	def __init__(self, epoch, wind_model=None, grav_model=gravity.ConstantGravityModel, atm_model=atmosphere.ISA, mag_model=geomagnetic.WMM, logger=None):
		"""
		Initializes the rocket environment (wind, atmosphere, ...) and models.

		Parameters :
		------------
			epoch
				Simulation start epoch (in years). This value is used to synchronize each model to the same starting date.

			wind_model : wind.WindModelBase (optional)
				WindModel instance used to compute the wind velocity vector. (default value = None)

			grav_model : gravity.GravityModelBase (optional)
				GravityModel instance used to compute the gravitational acceleration vector. (default value = gravity.ConstantGravityModel)

			atm_model : atmosphere.AtmosphericModelBase (optional)
				AtmosphericModel instance used to compute the thermodynamic properties of the atmosphere. (default value = atmosphere.ISA)

			mag_model : geomagnetic.GeoMagneticModelBase (optional)
				GeoMagneticModel instance used to compute the magnetic vector. (default value = geomagnetic.WMM)

			logger : logging
				logging class used to log the status of the computations
				
		"""
		self.logger = logger

		if not issubclass(grav_model, gravity.GravityModelBase): raise ValueError(f"gravity_model must be a subclass of {gravity.GravityModelBase}.")
		if not issubclass(mag_model, geomagnetic.GeoMagneticModelBase): raise ValueError(f"mag_model must be a subclass of {geomagnetic.GeoMagneticModelBase}.")

		if wind_model is not None: 
			if not issubclass(wind_model, wind.WindModelBase): raise ValueError(f"wind_model must be a subclass of {wind.WindModelBase} or {np.ndarray}")

			self._wind_model = wind_model
		else:
			v_wind = 0
			self._wind_model = wind.WindModelTables(epoch, np.array([0.0, 10000.0]), [v_wind, v_wind], np.zeros(2), logger=logger) # no wind

		# initializes the different models used
		self._gravity_model = grav_model(epoch, logger=logger)
		self._atmospheric_model = atm_model(epoch, logger=logger)
		self._mag_model = mag_model(epoch, logger=logger)

		if self.logger is not None:
			self.logger.info("Environment initialized :")
			self.logger.info("gravity model {self._gravity_model}")
			self.logger.info("atmospheric model {self._atmospheric_model}")
			self.logger.info("geomagnetic model {self._mag_model}")

		self._epoch = epoch

	@property
	def wind_model(self):
		"""
		Returns the wind model used throughout the simulations
		"""
		return self._wind_model

	@property
	def gravity_model(self):
		"""
		Returns the gravity model used throughout the simulations
		"""
		return self._gravity_model

	@property
	def atmospheric_model(self):
		"""
		Returns the atmospheric model used throughout the simulations
		"""
		return self._atmospheric_model

	@property
	def geomagnetic_model(self):
		"""
		Returns the geomagnetic model used throughout the simulations
		"""
		return self._mag_model
	
	def get_wind_velocity(self, t, r):	
		"""
		Computes the gravitational acceleration vector in the ecef frame.

		Parameters :
		------------
			r : numpy.ndarray (3, N)
				ECEF positions at which the gravitational acceleration is to be computed
			
			r : float/numpy.ndarray (N,)
				time in seconds at which the wind velocity is to be computed

		Returns :
		---------
			winc_vel : numpy.ndarray (3, N)
				wind velocity vetor in meters per second
		"""
			
		return self._wind_model.velocity(r, t)

	def g(self, r):
		"""
		Computes the gravitational acceleration vector in the ecef frame.

		Parameters :
		------------
			r : numpy.ndarray (3, N)
				ECEF positions at which the gravitational acceleration is to be computed

		Returns :
		---------
			g : numpy.ndarray (3, N)
				gravitational acceleration vector in the ecef frame
		"""
		return self._gravity_model.g(r)

	def __str__(self):
		"""
		Logs a description of the class into the terminal
		"""
		print("Environment simulation class :")
		print(f"gravity model : {self._gravity_model}")
		print(f"atmospheric model : {self._atmospheric_model}")
		print(f"geomagnetic model : {self._mag_model}")
		return ""


	@property
	def epoch(self):
		"""
		Get simulation epoch
		"""
		return self._epoch		