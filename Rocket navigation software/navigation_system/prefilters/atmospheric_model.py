# -*- coding: utf-8 -*-
"""
Created on Sat Feb 26 11:13:59 2022

@author: Thomas Maynadié
"""

import numpy as np
import functools

class vectorize(np.vectorize):
    def __get__(self, obj, objtype):
        return functools.partial(self.__call__, obj)
        
class ISA():
    def __init__(self):
        # setup ISA tables
        self.altitude_table = np.array([0, 11000, 20000, 32000, 47000])
        self.pressure_table = np.array([101325, 22632.1, 5474.89, 868.019, 110.906])
        self.temperature_table = np.array([288.15, 216.65, 216.65, 228.65, 270.65])
        self.temperature_gradient_table  = np.array([-0.0065, 0, 0.001, 0.0028])
        
        self.n_points_isa = len(self.altitude_table)
        
        # setup other variables
        self.g = 9.80665
        self.R = 287.04
    
    '''
        Retreives the altitude from given pressure values
    '''
    def get_altitude(self, pressure):
        pressure = np.asarray(pressure)
        region_index = self.__get_atmospheric_region_index(pressure, vartype='p')
        
        altitude = self.__invert(pressure, region_index)
        
        if(np.size(altitude) == 1): return altitude.item()
        
        return altitude
    
    '''
        Retreives ISA atmospheric values for given altitude values
    '''
    def get_predictions(self, z):
        z = np.asarray(z)
        region_index = self.__get_atmospheric_region_index(z, vartype='z')
        
        pressure, temperature = self.__predict(z, region_index)
        
        if(np.size(pressure) == 1): return pressure.item(), temperature.item()
        
        return pressure, temperature
    
    @vectorize
    def __invert(self, pressure, region):
        Pk = self.pressure_table[region]
        Tk = self.temperature_table[region]
        zk = self.altitude_table[region]
        Tzk = self.temperature_gradient_table[region]
        
        if region == 1:
            dz = (Tzk/(self.g*self.R))*np.log(Pk/pressure)  
        else:
            dz = Tk/Tzk*((pressure/Pk)**(-Tzk*self.R/self.g)-1)
    
        return zk + dz
    
    @vectorize
    def __predict(self, z, region):
        Pk = self.pressure_table[region]
        Tk = self.temperature_table[region]
        zk = self.altitude_table[region]
        Tzk = self.temperature_gradient_table[region]
        
        if(region == 1): 
            temperature = Tk
            pressure = Pk * np.exp(-self.g*(z - zk)/(self.R*Tk))

        else: 
            temperature = Tk + (z - zk) * Tzk
            pressure = Pk * (temperature/Tk) ** (-self.g/(self.R*Tzk))
       
        return pressure, temperature
    
    @vectorize
    def __get_atmospheric_region_index(self, value, vartype='z'): 
        if (vartype == 'p'): table_values = self.pressure_table
        elif (vartype == 'z'): table_values = self.altitude_table
        else: raise("ERROR : unknown vartype for __get_atmospheric_region_index")
        
        regions = np.nonzero(table_values > value)[0]
        assert len(regions)>0, "Please provide an altitude within the range of the ISA table : " + str(value)

        return regions[0]-1