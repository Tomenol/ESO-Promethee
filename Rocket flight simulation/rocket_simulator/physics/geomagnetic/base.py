from abc import ABC, abstractmethod

"""
This module defines the structure of Earth's magnetic field models.
"""

class GeoMagneticModelBase(ABC):
	"""
	Base class used to construct geomagnetic models
	"""
	def __init__(self, epoch, logger=None):
		"""
		Initializes the specific geomagnetic model.

		Parameters :
		------------
			epoch : float
				Reference date from which the measurements of the magnetic field are performed (in years [from 2020.0 - 2025.0])

			logger : logging.Logger
				Logger instance used to log the status of the computations
		"""
		self._epoch = epoch
		self.logger = logger

	@abstractmethod
	def get_magnetic_vector_enu(self, r, t):
		"""
		Compute magnetic vector in the ENU frame of reference.

		Parameters :
		------------
			r : float/numpy.ndarray (3, N)
				position vector of the measurement point in the ECEF frame of reference

			t : float/numpy.ndarray (N,)
				time of the measurement with respect to the start of the epoch (in seconds)

		Returns :
		---------
			mag_enu : float/numpy.ndarray (3, N)
				magnetic vector (in nT) in the ENU frame.
		"""
		pass

	@abstractmethod
	def get_magnetic_vector_ecef(self, r, t):
		"""
		Compute magnetic vector in the ECEF frame of reference.

		Parameters :
		------------
			r : float/numpy.ndarray (3, N)
				position vector of the measurement point in the ECEF frame of reference

			t : float
				time of the measurement with respect to the start of the epoch (in seconds)

		Returns :
		---------
			mag_ecef : float/numpy.ndarray (3, N)
				magnetic vector (in nT) in the EECEFNU frame.
		"""
		pass
 	
	@property
	def epoch(self):
		return self._epoch

	def __str__(self):
		print("geomagnetic model :")
		print("epoch : ", self._epoch)
		return ""