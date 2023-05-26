from abc import ABC, abstractmethod

class WindModelBase(ABC):
	"""
	Abstract class used to define the wind model used during the simulation.
	"""
	def __init__(self, epoch, logger=None):
		"""
		Abstract class used to define the wind model used during the simulation.

		Parameters :
		------------
			epoch : float
				starting simulation date in years.

			logger : logging
				logging class used to log computation status 
		"""
		self.logger = logger
		self._epoch = epoch

	@abstractmethod
	def velocity(self, r, t):
		"""
		Computes the velocity wind at the given position and time.

		Parameters :
		------------

		r : np.ndarray (3,)
			ECEF position at which the wind velocity is to be computed.

		t : float
			time in seconds at which the wind velocity is to be computed.

		Returns :
		---------

			np.ndarray : (3,)
				wind velocity vector in the ECEF frame
		"""
		pass

	@property
	def epoch(self):
		return self._epoch