import numpy as np

# world magnetic model
import wmm2020

from . import base
from ...common import frames

class WMM(base.GeoMagneticModelBase):
	"""
	Computes the magetic vector using the 2020 World Magnetic Model.
	"""
	def __init__(self, epoch, logger=None):
		"""
		Initializes the 2020 World Magnetic Model.

		Parameters :
		------------
			epoch : float
				Reference date from which the measurements of the magnetic field are performed (in years [from 2020.0 - 2025.0])

			logger : logging.Logger
				Logger instance used to log the status of the computations
		"""
		super().__init__(epoch, logger=None)

		if self.logger is not None:
			self.logger.debug(f"WMM Initialized, epoch = {epoch} yrs.")

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
		date = self._epoch + t/(3600*24*365.25) # date in years
		lon, lat, z = frames.ecef_to_geodetic(r, deg=True)

		# compute WMM solution
		wmm_solution = wmm2020.wmm(lon, lat, z*1e-3, date)

		# inclination and declination in degrees
		# incl = wmm_solution["incl"][0, 0].data*np.pi/180.0
		# decl = wmm_solution["decl"][0, 0].data*np.pi/180.0

		# get magnetic field components
		mag_e = wmm_solution["east"][0].data
		mag_n = wmm_solution["north"][0].data
		mag_d = wmm_solution["down"][0].data

		mag_enu = np.array([mag_e, mag_n, -mag_d]).reshape(3, -1)

		if self.logger is not None:
			self.logger.debug(f"WMM -> computing magnetic vector {mag_enu} (enu) at lon = {lon} deg, lat = {lat} deg.")

		return mag_enu

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
		mag_enu = self.get_magnetic_vector_enu(r, t)
		mag_ecef = frames.enu_to_ecef(mag_enu, r, only_rotation=True)

		if self.logger is not None:
			self.logger.debug(f"WMM -> converting magnetic vector to ecef {mag_ecef}")

		return mag_ecef

	def __str__(self):
		"""
		Prints class data and informations
		"""
		print("{WMM2020 Geomagnetic Model (NOAA, NCEI, ...), epoch=", self.epoch, "yrs.}")
		return ""