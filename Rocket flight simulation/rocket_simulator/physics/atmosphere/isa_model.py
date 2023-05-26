import numpy as np

from . import base
from .. import earth 

"""
International Standard Atmosphere :

	Computes the thermodynamic properties of the atmosphere (Temperature, Pressure and Density) as a function of altitude z.
"""

class ISA(base.AtmosphericModelBase):
	"""
	Implementation of the ISA model.
	"""
	def __init__(self, epoch, logger=None):
		"""
		International Standard Atmosphere :

			This class implements the ISA model which allows the user to compute the thermodynamic properties of the atmosphere (Temperature, Pressure and Density) 
			as a function of altitude z.

			The algorithm implements the basic equations governing the evolution of T(z), P(z) and rho(z) :

			.. math::
				T(z) = T_{0} + T_{z} (z - z_{0})
			.. math::
				dP = -\\rho g dz
			.. math::	
				\\rho = \\frac{P}{rT_{z}}

			the values of .. math:: T_{0}, P_{0}, z_{0} and T_{z} are tabulated according to the ISA standard.
		
		Parameters :
		------------
			epoch : float
				Reference date from which the measurements of the magnetic field are performed (in years [from 2020.0 - 2025.0])

			logger : logging.Logger
				Logger instance used to log the status of the computations


		Reference : U.S. Standard atmosphere, 1976 (https://www.ngdc.noaa.gov/stp/space-weather/online-publications/miscellaneous/us-standard-atmosphere-1976/us-standard-atmosphere_st76-1562_noaa.pdf)
		"""

		super().__init__(epoch, logger=logger)

		# set up atmospheric tables
		self.altitude = np.array([0.0, 11019.0, 20063.0, 32162.0, 47350.0, 51413.0, 7180.0, 86000.0]) 	# altitude
		self.Tz = np.array([-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0.0, -2.8e-3, -2.0e-3])						# Lapse rate (dT/dz)
		self.T0 = 273.15 + np.array([15.0, -56.5, -56.5, -44.5, -2.5, -2.5, -58.5, -86.2])				# Temperature
		self.P0 = np.array([101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734])			# Pressure

		# perfect gas constant (r = R/M)
		self.r = 287.05

		self.rho0 = self.P0/(self.r*self.T0)															# density

		if self.logger is not None:
			self.logger.info("Atmospheric model -> ISA model initialized")

	def rho(self, z, t, g=earth.G0):
		"""
		Computes atmospheric density as a function of altitude according to the ISA model.

		Parameters :
		------------

			z : float/numpy.ndarray (N,)
				Geopotential altitude in meters. 

			t : float/numpy.ndarray (N,)
				Time in seconds (not needed in the ISA model).

			g : float (optional)
				Gravitational acceleration. (default value : g = g0 = 9.81 m/s^2)

		Returns :
		---------

			float/numpy.ndarray (N,) :
				Atmospheric density in kg/m^3
		"""
		pressure = self.P(z, t, g=g)
		temperature = self.T(z, t)

		rho = pressure/(self.r * temperature)

		if self.logger is not None:
			self.logger.debug(f"Atmospheric model -> computing density : (z = {z} m, g = {g} m/s2, t = {t} K) -> rho = {rho} kg/m3")

		return rho

	def P(self, z, t, g=earth.G0):
		"""
		Computes atmospheric pressure as a function of altitude according to the ISA model.

		Parameters :
		------------

			z : float/numpy.ndarray (N,)
				Geopotential altitude in meters. 

			t : float/numpy.ndarray (N,)
				Time in seconds (not needed in the ISA model).

			g : float (optional)
				Gravitational acceleration. (default value : g = g0 = 9.81 m/s^2)

		Returns :
		---------

			float/numpy.ndarray (N,) :
				Atmospheric pressure in Pa.
		"""
		if not z <= self.altitude[-1]:
			raise ValueError(f"The altitude must e between {self.altitude[0]} and {self.altitude[-1]}")

		if z < 0: z = 0
		if z in self.altitude:
			if self.logger is not None: 
				self.logger.debug(f"Atmospheric model -> computing pressure on known tabulated point.")

			i = np.where(self.altitude == z)[0][0]

			pressure = self.P0[i]
		else:

			i = np.where(self.altitude >= z)[0][0]-1

			if self.Tz[i] != 0:
				pressure = self.P0[i]*(1 + self.Tz[i]/self.T0[i]*(z - self.altitude[i]))**(-g/(self.Tz[i]*self.r))
			else:
				pressure = self.P0[i]*np.exp(-g*(z - self.altitude[i])/(self.T0[i]*self.r))

		if self.logger is not None:
			self.logger.debug(f"Atmospheric model -> computing atmospheric pressure : (z = {z} m, g = {g}m/s2, t = {t} K) -> P = {pressure} Pa.")

		return pressure

	def T(self, z, t):
		"""
		Computes atmospheric temperature as a function of altitude according to the ISA model.

		Parameters :
		------------

			z : float/numpy.ndarray (N,)
				Geopotential altitude in meters. 

			t : float/numpy.ndarray (N,)
				Time in seconds (not needed in the ISA model).

		Returns :
		---------

			float/numpy.ndarray (N,) :
				Atmospheric temperature in Kelvins.
		"""
		if not z <= self.altitude[-1]:
			raise ValueError(f"The altitude must be between {self.altitude[0]} and {self.altitude[-1]}")

		if z < 0: z = 0
		if z in self.altitude:
			if self.logger is not None: 
				self.logger.debug(f"Atmospheric model -> computing temperature on known tabulated point.")

			i = np.where(self.altitude == z)[0][0]

			temperature = self.T0[i]
		else:
			i = np.where(self.altitude >= z)[0][-1]-1

			temperature = self.T0[i] + self.Tz[i] * (z - self.altitude[i])

		if self.logger is not None:
			self.logger.debug(f"Atmospheric model -> computing temperature pressure : (z = {z} m, t = {t} s) -> T = {temperature} K.")

		return temperature

	def rho_inv(self, rho, t, g=earth.G0):
		"""
		Invert density model.

		TODO : NOT TESTED
		"""
		if not (rho >= self.rho0[0] and rho <= self.rho0[-1]):
			raise ValueError(f"The density rho must be between {self.rho0[0]} and {self.rho0[-1]}")

		if rho in self.rho0:
			i = np.where(self.rho0 == rho)[0][-1]

			z = self.altitude[i]
		else:
			i = np.where(self.rho0 >= rho)[0][-1]

			if self.Tz[i] != 0:
				z = self.altitude[i] + 1/self.Tz[i]*((rho*self.r*self.Tz[i]**(-g/(self.r*self.Tz[i]))/self.P0[i])**(-1/(1+g/(self.r*self.Tz[i]))) - self.T0[i])
			else:
				z = self.altitude[i] - np.log(P/self.P0[i])*(r*self.T0[i]/g)

		return z

	def P_inv(self, P, t, g=earth.G0):
		"""
		Invert pressure model.

		TODO : NOT TESTED
		"""
		if not (P >= self.P0[0] and P <= self.P0[-1]):
			raise ValueError(f"The pressure must be between {self.P0[0]} and {self.P0[-1]}")

		if P in self.P0:
			i = np.where(self.P0 == P)[0][-1]

			z = self.altitude[i]
		else:
			i = np.where(self.P0 >= P)[0][-1]

			if self.Tz[i] != 0:
				z = self.altitude[i] - self.Tz[i]/self.T0[i] + (P/self.P0[i])**(-r*self.Tz[i]/g)
			else:
				z = self.altitude[i] - np.log(rho*r*self.T0[i]/self.P0[i])*(self.r*self.T0[i]/g)

		return z

	def T_inv(self, T, t):
		"""
		Invert temperature model.

		TODO : NOT TESTED
		"""
		if not (T >= self.T0[0] and T <= self.T0[-1]):
			raise ValueError(f"The temperature must be between {self.T0[0]} and {self.T0[-1]}")

		if T in self.T0:
			i = np.where(self.T0 == T)[0][-1]

			z = self.altitude[i]
		else:
			i = np.where(self.T0 >= T)[0][-1]

			if self.Tz[i] != 0:
				z = self.altitude[i] + (T - self.T0[i])/self.Tz[i]
			else:
				z = self.altitude[i]

		return z

	def __str__(self):
		"""
		Prints class data and informations
		"""
		print("{International Standard Atmospheric (ISA) Model, 1976}")
		return ""