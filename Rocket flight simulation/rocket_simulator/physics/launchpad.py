from . import environment

from ..common import quaternion
from ..common import frames

import numpy as np

class LaunchPad(object):
	"""
	Holds the characteristics of the launchpad.
	"""
	def __init__(self, lon, lat, height, env, initial_attitude_euler=np.zeros(3), deg=False, logger=None):
		"""
		Initialization of the launchpad characteristics.

		Parameters :
		------------
			lon : 	
				Longitude of the launchpad (angle deg/radians)
			
			lat :
				Latitude of the launchpad (angle deg/radians)
			
			height :
			 	Altitude of the launchpad (in meters).
			
			env : environment.Environment
				Environment instance used to model the surroundings of the rocket (atmophere, wind, ...)
			
			initial_attitude_euler : numy.ndarray 
				Initial launch attitude of the rocket with respect to the ENU frame of reference
			
			deg : bool (optional)
				if True, all the angles are in degrees (default value = False).
			
			logger : logging.Logger (optional)
				Logger instance used to log the status of the class (default value = None)
		"""
		if not isinstance(env, environment.Environment):
			raise AttributeError(f"env must be an instance of {environment.Environment}")

		self.logger = logger

		self.env = env
		self.ecef = frames.geodetic_to_ecef(lon, lat, height, deg=deg)

		q = quaternion.Quaternion(*initial_attitude_euler, deg=deg)
		#q = frames.quaternion_ecef_to_enu()*quaternion.Quaternion(0, lon, 0, deg=deg)*quaternion.Quaternion(-lat, 0, 0, deg=deg)*q
		self.initial_attitude = q.normalize().convert_to_attitude_vector(deg=False)

		if self.logger is not None:
			self.logger.info("setting launchpad :")
			self.logger.info(f"launch attitude : {self.initial_attitude*180.0/np.pi} [deg]")
			self.logger.info(f"launch dir ecef : {np.dot(quaternion.Quaternion(*self.initial_attitude, deg=False).DCM(), np.array([1, 0, 0]))} [deg].")
			self.logger.info(f"launch position (ecef) : {self.ecef}")

	def get_forces(self, F, r, t, t0):
		"""
		Computes the reaction forces of the launchpad.

		Parameters :
		------------
			F : numpy.ndarray (3,)
				sum of all the external forces exerted on the rocket (weight, ...) appart from the reaction forces

			r : numpy.ndarray (3,)
				ecef coordinates of the rocket.

			t : numpy.ndarray (3,)
				current time in seconds.
				
			t0 : float
				launch date (in seconds) with respect to the starting epoch

		Returns :
		---------
			lp_forces : numpy.ndarray (3,)
				reaction forces of the launchpad applied on the rocket
		"""
		if t > t0:
			lp_forces = np.zeros(3)
		else:
			lp_forces = -F

		if self.logger is not None:
			self.logger.debug(f"computing launchpad reaction forces {lp_forces}.")

		return lp_forces

	def get_launch_position_ecef(self):
		"""
		Computes the initial position of the rocket in the ECEF frame

		Parameters :
		------------
			None

		Returns :
		---------
			ecef : numpy.ndarray (3,)
				ECEF position of the launchpad
		"""
		return self.ecef

	def get_launch_attitude(self, deg=False):
		"""
		Computes the initial launch attitude of the rocket

		Parameters :
		------------
			deg : float (optional)
				if True, all the angles are in degrees (default value = False).

		Returns :
		---------
			ecef : numpy.ndarray (3,)
				initial attitude (degrees/radians) of the rocket at launch
		"""
		if deg is True:
			return self.initial_attitude*180.0/np.pi
		else:
			return self.initial_attitude

	def __str__(self):
		print("launch attitude : ", self.get_launch_attitude(deg=True))		
		print("launch ecef : ", self.ecef)
		return ""