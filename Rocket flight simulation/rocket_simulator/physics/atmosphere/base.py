from abc import ABC, abstractmethod

class AtmosphericModelBase(ABC):
	"""
	Base atmospheric model class
	"""
	def __init__(self, epoch, logger=None):
		"""
		Initializes the atmospheric model.
		
		Parameters :
		------------
			epoch : float
				Reference date from which the measurements of the magnetic field are performed (in years [from 2020.0 - 2025.0])

			logger : logging.Logger
				Logger instance used to log the status of the computations
		"""
		self.logger = logger
		self._epoch = epoch
		
	@abstractmethod
	def rho(self, z, t):
		"""
		Computes atmospheric density as a function of altitude.

		Parameters :
		------------
			z : float/numpy.ndarray (N,)
				Geopotential altitude in meters. 

			t : float/numpy.ndarray (N,)
				Time in seconds.

			g : float (optional)
				Gravitational acceleration. (default value : g = g0 = 9.81 m/s^2)

		Returns :
		---------
			float/numpy.ndarray (N,) :
				Atmospheric density in kg/m^3
		"""
		pass

	@abstractmethod
	def P(self, z, t):
		"""
		Computes atmospheric pressure as a function of altitudel.

		Parameters :
		------------
			z : float/numpy.ndarray (N,)
				Geopotential altitude in meters. 

			t : float/numpy.ndarray (N,)
				Time in seconds.

		Returns :
		---------
			float/numpy.ndarray (N,) :
				Atmospheric pressure in Pa.
		"""
		pass

	@abstractmethod
	def T(self, z, t):
		"""
		Computes atmospheric temperature as a function of altitudel.

		Parameters :
		------------
			z : float/numpy.ndarray (N,)
				Geopotential altitude in meters. 

			t : float/numpy.ndarray (N,)
				Time in seconds (not needed in the ISA model).

		Returns :
		---------
			float/numpy.ndarray (N,) :
		"""
		pass

	@property
	def epoch(self):
		return self._epoch

	def __str__(self):
		"""
		Prints class data and informations
		"""
		print("Undefined atm model")
		return ""