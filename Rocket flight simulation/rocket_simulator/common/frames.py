import numpy as np

from ..physics import earth
from . import quaternion

def geodetic_to_ecef(longitude, latitude, altitude, deg=False):
	"""
	Coordinate frame transformation from geodetic (longitude, latitude, altitude) to the ECEF frame (Earth Centered Earth Fixed frame)

	Parameters :
	------------
		longitude : float/np.ndarray (N,)
			Longitude of the point(s) in the geodetic frame to be converted to the ECEF frame.

		latitude : float/np.ndarray (N,)
			Latitude of the point(s) in the geodetic frame to be converted to the ECEF frame.

		altitude : float/np.ndarray (N,)
			Altitude of the point(s) over the reference geoid (WGS84) to be converted to the ECEF frame.

		deg : bool (optional)
			If True, all the angles are in degrees. If False, (default value) all the angles are in radians.

	Returns :
	---------
		np.ndarray shape (3, N)
			Points in the ECEF frame (x, y, z).
	
	Reference : J. Zhu, Exact conversion of earth-centered, earth-fixed coordinates to geodetic coordinates (DOI : https://doi.org/10.1109/7.303772)
	"""
	longitude = np.asfarray(longitude)
	latitude = np.asfarray(latitude)
	altitude = np.asfarray(altitude)

	if np.size(longitude) != np.size(latitude) and np.size(longitude) != np.size(altitude): raise ValueError("arrays longitude, latitude and altitude must be of the same size.")

	if deg is True:
		longitude = longitude*np.pi/180.0
		latitude = latitude*np.pi/180.0

	f = earth.POLAR_RADIUS/earth.EQUATORIAL_RADIUS
	e = np.sqrt(1 - f**2)
	N = earth.EQUATORIAL_RADIUS/np.sqrt(1 - e**2 * np.sin(latitude)**2)

	x = (N + altitude) * np.cos(latitude) * np.cos(longitude)	
	y = (N + altitude) * np.cos(latitude) * np.sin(longitude)
	z = (N*f**2 + altitude) * np.sin(latitude)

	return np.array([x, y, z], dtype=float)

def ecef_to_enu(ecef, ref_ecef, only_rotation=True):
	"""
	Coordinate frame transformation from ECEF (longitude, latitude, altitude) to the ENU frame (East, North, Up).
	The algorithm first converts the origin of the ENU reference frame to the geodetic frame, and then computes the rotation matrix
	to convert the ECEF vectors to ENU vectors.
	if only_rotation is False, then the resulting position vectors will be expressed with respect to the origin of the ENU reference frame. 

	Parameters
	----------
		ecef : float/np.ndarray (3, N)
			x, y, z coordinates of the ECEF vectors to be converted to ENU

		ref_ecef : float/np.ndarray (3, N)
			x, y, z coordinates of the ENU reference frame origin in the ECEF frame 

		only_rotation : bool (optional)
			If True, the algorithm only performs a rotation of the ECEF vector to the ENU frame without moving the origin of the vector to 
			the origin of the ENU reference frame. For example, set only_rotation=True when dealing with velocity vectors to ensure 
			the conservation of the vector's magnitude.

	Returns
	-------
		enu : np.ndarray shape (3, N)
			Points in the ENU frame (x, y, z).
	"""
	ecef = np.asfarray(ecef).reshape((3, -1))
	ref_ecef = np.asfarray(ref_ecef).reshape((3, -1))

	lon, lat, h = ecef_to_geodetic(ref_ecef, deg=False)

	M = np.array([
		[-np.sin(lon), 							 np.cos(lon),  			 			np.zeros(np.size(lon))	],
		[-np.sin(lat)*np.cos(lon), 				-np.sin(lat)*np.sin(lon),  			np.cos(lat)			],
		[ np.cos(lat)*np.cos(lon), 	   			 np.cos(lat)*np.sin(lon),			np.sin(lat)			]], dtype=float)

	if only_rotation is False:
		ecef = ecef - ref_ecef

	enu = np.einsum("ijk,jk->ik", M, ecef)

	if np.size(enu) == 3:
		enu = np.reshape(enu, (3,))
	
	return enu

def enu_to_ecef(enu, ref_ecef, only_rotation=True):
	"""
	Coordinate frame transformation from ENU frame (East, North, Up) to the ECEF (Earth Centered Earth Fixed frame).
	The algorithm first converts the origin of the ENU reference frame to the geodetic frame, and then computes the rotation matrix
	to convert the ENU vectors to ECEF vectors.
	if only_rotation is False, then the resulting position vectors will be expressed with respect to the origin of the ECEF reference frame. 

	Parameters
	----------
		enu : float/np.ndarray (3, N)
			x, y, z coordinates of the ENU vectors to be converted to ECEF

		ref_ecef : float/np.ndarray (3, N)
			x, y, z coordinates of the ENU reference frame origin in the ECEF frame 

		only_rotation : bool (optional)
			If True, the algorithm only performs a rotation of the ENU vector to the ECEF frame without moving the origin of the vector to 
			the origin of the ENU reference frame. For example, set only_rotation=True when dealing with velocity vectors to ensure 
			the conservation of the vector's magnitude.

	Returns
	-------
		ecef : np.ndarray shape (3, N)
			Points in the ECEF frame (x, y, z).
	"""
	enu = np.asfarray(enu).reshape((3, -1))
	ref_ecef = np.asfarray(ref_ecef).reshape((3, -1))

	lon, lat, h = ecef_to_geodetic(ref_ecef, deg=False)

	M = np.array([
		[-np.sin(lon), 				-np.sin(lat)*np.cos(lon),  			np.cos(lat)*np.cos(lon)],
		[ np.cos(lon), 				-np.sin(lat)*np.sin(lon),  			np.cos(lat)*np.sin(lon)],
		[ np.zeros(np.size(lon)), 	 np.cos(lat),  						np.sin(lat)]], dtype=float)

	ecef = np.einsum("ijk,jk->ik", M, enu)

	if only_rotation is False:
		ecef = ecef + ref_ecef

	if np.size(ecef) == 3:
		ecef = np.reshape(ecef, (3,))

	return ecef


def ecef_to_geodetic(ecef, deg=False):
	"""
	Coordinate frame transformation from the ECEF frame (Earth Centered Earth Fixed frame) to the geodetic (longitude, latitude, altitude) frame.

	Parameters :
	------------
		ecef : float/np.ndarray (N,)
			Points in the ECEF frame (x, y, z) to be converted to the geodetic.

		deg : bool (optional)
			If True, all the angles are in degrees. If False, (default value) all the angles are in radians.

	Returns :
	---------
		np.ndarray shape (3,N)
			longitude, latitude and altitude of the points with respect to the reference geoid (WGS84)
	
	Reference : J. Zhu, Exact conversion of earth-centered, earth-fixed coordinates to geodetic coordinates (DOI : https://doi.org/10.1109/7.303772)
	"""
	ecef = np.asfarray(ecef)

	x = ecef[0]
	y = ecef[1]
	z = ecef[2]

	e2 = (earth.EQUATORIAL_RADIUS**2 - earth.POLAR_RADIUS**2)/earth.EQUATORIAL_RADIUS**2

	p = np.sqrt(x**2 + y**2)
	F = 54.0*earth.POLAR_RADIUS**2*z**2
	G = p**2 + (1-e2)*z**2 - e2*(earth.EQUATORIAL_RADIUS**2-earth.POLAR_RADIUS**2)

	c = e2**2*F*p**2/G**3
	s = (1 + c + np.sqrt(c**2 + 2*c))**(1.0/3.0)
	del c

	k = s + 1 + 1/s
	del s

	P = F/(3*k**2*G**2)
	del k, F, G

	Q = np.sqrt(1 + 2*e2**2*P)

	r0 = -P*e2*p/(1+Q) + np.sqrt(0.5*earth.EQUATORIAL_RADIUS**2*(1+1/Q) - P*(1-e2)*z**2/(Q*(1+Q)) - 0.5*P*p**2)
	del P, Q

	U = np.sqrt((p-e2*r0)**2 + z**2)
	V = np.sqrt((p-e2*r0)**2 + (1-e2)*z**2)
	del r0, e2

	z0 = earth.POLAR_RADIUS**2*z/(earth.EQUATORIAL_RADIUS*V)
	altitude = U * (1-earth.POLAR_RADIUS**2/(earth.EQUATORIAL_RADIUS*V))
	del U, V

	ep2 = (earth.EQUATORIAL_RADIUS**2 - earth.POLAR_RADIUS**2)/earth.POLAR_RADIUS**2
	latitude = np.arctan((z+ep2*z0)/p)
	del p, ep2, z0

	longitude = np.arctan2(y, x)

	if deg is True:
		longitude = longitude*180.0/np.pi
		latitude = latitude*180.0/np.pi

	return longitude, latitude, altitude


def quaternion_ecef_to_enu():
	"""
	Computes the rotation quaternion from the ECEF to the ENU reference frame.

	Parameters :
	------------
		None

	Returns :
	---------
		quaternion.Quaterion :
			rotation quaternion from the ECEF to the ENU reference frame.
	"""
	return quaternion.Quaternion(90, 0, 0, deg=True)*quaternion.Quaternion(0, 90, 0, deg=True)



def aero_to_body_frame(vector, alpha, beta, deg=False):
	"""
	Converts a vector in the aerodynamic frame to the body frame of reference.

	Parameters :
	------------
		vector : np.ndarray (3, N)
			Vector (aerodynamic frame) coordinates to be transformed. If multiple vectors are computed, the vectors shall be stacked by columns
		
		alpha : float
			Angle of attack (angle from the projection of the velocity vector in the xz plane (body frame) to the x-axis of the body reference frame)

		beta : float
			Sideslip angle (angle from the projection of the velocity vector in the xy plane (body frame) to the x-axis of the body reference frame)

		deg : bool (optional)
			If True, all the angles are in degrees. If False, (default value) all the angles are in radians.

	Returns :
	---------
		np.ndarray shape (3,N)
			x, y, z coordinates of the transformed vector to the body reference frame
	"""
	vector = np.asfarray(vector)

	if (np.shape(vector)[0]) != 3:
		vector = np.reshape(vector, (3, -1))

	# convert the angles to radians
	if deg is True:
		alpha 	= alpha*np.pi/180.0
		beta 	= beta*np.pi/180.0

	# compute aero->body rotation matrix
	M_aero_to_body = np.array([
		[  np.cos(alpha)*np.cos(beta), 		-np.sin(beta), 					-np.sin(alpha)*np.cos(beta)	], 
		[  np.cos(alpha)*np.sin(beta), 	 	 np.cos(beta), 					-np.sin(alpha)*np.sin(beta)	],
		[  np.sin(alpha), 		 			 np.zeros(np.size(vector[0])), 	 	 np.cos(alpha)			]], dtype=float)

	# frame transformation
	if np.size(vector) == 3:
		vector_body_frame = np.einsum("ij,j->i", M_aero_to_body, vector)
	else:
		vector_body_frame = np.einsum("ijk,jk->ik", M_aero_to_body, vector)

	return vector_body_frame