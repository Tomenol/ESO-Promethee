import numpy as np

from ...physics import earth
from . import base 
"""
This module defines the constant gravity model g(r) = g.
"""

class ConstantGravityModel(base.GravityModelBase):
	"""
	Computes the gravitational acceleration considering that g(r) is constant.
	"""
	def __init__(self, epoch, g=earth.G0, logger=None):
		"""
		Computes the gravitational acceleration considering that g(r) is constant.
		"""
		super().__init__(epoch, logger=logger)
		self._g = g

		if self.logger is not None:
			self.logger.info(f"ConstantGravityModel initialized : g0 = {g}")

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
		r_hat = r/np.linalg.norm(r)

		return -self._g * r_hat

	def __str__(self):
		"""
		Prints class data and informations
		"""
		print("{Constant Gravity Model, g0=", self._g, " m/s2}")
		return ""