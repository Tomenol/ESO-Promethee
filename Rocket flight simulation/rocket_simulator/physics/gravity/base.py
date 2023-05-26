from abc import ABC, abstractmethod

"""
This module defines the structure of Earth's gravitational field models.
"""

class GravityModelBase(ABC):
	"""
	Abstract base class defining the implementation of a model which computes the gravitational acceleration due to the Earth's gravitational field.
	"""
	def __init__(self, epoch, logger=None):
		"""
		Gravity model abstract base class constructor.

		Parameters :
		------------
			epoch : float
				starting simulation date in years.

			logger : logging
				logging class used to log computation status 
		"""
		self._epoch = epoch
		self.logger = logger


	@abstractmethod
	def g(self, r):
		"""
		Computes the gravitational acceleration at a given ECEF position r.

		Parameters :
		------------
			r : np.ndarray (3,)
				ECEF position vector at which the gravitational acceleration is computed.

		Returns :
		---------
			gravitational_acceleration : np.ndarray (3,)
				acceleration vector due to the earth's gravitational field.
		"""
		pass

	@property
	def epoch(self):
		return self._epoch


