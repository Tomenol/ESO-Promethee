# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 16:17:40 2022

@author: Thomas Maynadié
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import matplotlib

from scipy import signal

def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False


# data_file = open("data//log_vol.txt", "r")

    
# raw_data = data_file.readlines()
# data_name = ["t_slave", "t", "ax1", "ay1", "az1", "ax2", "ay2", "az2", "wx1", "wy1", "wz1", "wx2", "wy2", "wz2", "mx1", "my1", "mz1", "mx2", "my2", "mz2", "P", "T", "zP", "lon", "lonDir", "lat", "latDir", "z", "V", "dir", "h", "min", "sec", "isNew", "fix"]
# types = []

# for name in data_name:
#     type_ = (name, 'f8')

#     types.append(type_)

# data = {data_name_i : [] for data_name_i in data_name}

# t0 = -1

# for line_data in raw_data[1:-1]:
#     if line_data != "\n":
#         line_data_arr = line_data.replace('*\n', "").split(sep=";")

#         for i, data_val in enumerate(line_data_arr):
#             if(i==0):
#                 if(t0 == -1):
#                     t0 =  float(data_val)
                    
#                 data[data_name[i]].append(float(data_val) - t0)
#             else:    
#                 if(i>=len(data_name)): break
                
#                 if isfloat(data_val): 
#                     data[data_name[i]].append(float(data_val))
#                 else: 
#                     data[data_name[i]].append(0.0)

# data_np = np.zeros(len(data["t"]), dtype=types)

# for name in data.keys():
#     print(name, ", ", len(data[name]))
#     print(data[name])
#     data_np[name] = data[name]

# np.save("flight_data.npy", data_np)

data_np_raw = np.load("flight_data.npy")

data_msk = np.logical_and(data_np_raw["t"]/1000.0 >= 4252.8, data_np_raw["t"]/1000.0 <= 4510.0)
data_np = data_np_raw[data_msk]

data_np["t"] = (data_np["t"] - data_np["t"][0])/1000.0
data_np["t_slave"] = (data_np["t_slave"] - data_np["t"][0])/1000.0

# Analysis    
data_to_analyze_arr = ["ax1", "ay1", "az1"]
labels = ["ax1", "ay1", "az1"]

units = ["m/s2", "m/s2", "m/s2"]

t_alt_max = 18.88+20
climb_mask = data_np["t"] <= t_alt_max

t_climb = data_np["t"][climb_mask]

# pressure altitude
pressure_climbing = data_np["P"][climb_mask]
temperature_climbing = data_np["T"][climb_mask] + 273.15

z_tarbes = 174 # m
T_tarbes_zero = 32 # deg
P_tarbes_zero = 101800 # pa

Tz = -6.5e-3

T_isa_tarbes = 288.18 + Tz*z_tarbes

dT_isa = T_isa_tarbes - T_tarbes_zero
dT_rocket = temperature_climbing[0] - T_tarbes_zero

temperature_outside = temperature_climbing - dT_rocket
T0_isa = 288.15 + dT_isa

P_isa_tarbes = 101325*(1 + Tz*z_tarbes/T0_isa)**5.2561
dP_isa = P_isa_tarbes - P_tarbes_zero
dP_rocket = pressure_climbing[0] - P_tarbes_zero
pressure_outside = pressure_climbing - dP_rocket

P0_isa = 101325 + dP_isa

z_temp = (temperature_outside - T0_isa)/Tz
z_press = (1.0 - ((pressure_outside)/P0_isa)**(1.0/5.2561))*T0_isa/Tz

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212)
ax1.plot(t_climb, z_press, "-k")


# acceleration altitude
fig = plt.figure()
ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)

accel = np.asfarray([
        [data_np["ax1"] + data_np["ax2"]], 
        [data_np["ay1"] + data_np["ay2"]], 
        [data_np["az1"] + data_np["az2"]], 
    ]).reshape((3,-1))/2.0

alpha0 = np.arctan(np.sqrt(accel[0][0]**2 + accel[1][0]**2)/accel[2][0])
print(alpha0)
alpha0=0

vz = np.zeros((len(t_climb)))
z = np.zeros((len(t_climb)))

for ti in range(1, len(t_climb)):
    dt = t_climb[ti] - t_climb[ti-1]

    vz[ti] = vz[ti-1] + accel[2][ti]*np.cos(alpha0)*dt
    z[ti] = z[ti-1] + vz[ti-1]*dt + accel[2][ti]*np.cos(alpha0)*dt**2/2

ax1.plot(t_climb, accel[2][climb_mask]*9.81, "-k")
ax2.plot(t_climb, vz, "-k")
ax3.plot(t_climb, z, "-k")
ax3.plot(t_climb, z_temp, "--k")
ax3.plot(t_climb, z_press, ":k")

ax1.set_xlabel("$t$ [$s$]")
ax1.set_ylabel("$\\gamma(t)$ [$m/s^2$]")      
ax1.grid() 

ax2.set_xlabel("$t$ [$s$]")
ax2.set_ylabel("$v_z(t)$ [$m/s$]")      
ax2.grid() 

ax3.set_xlabel("$t$ [$s$]")
ax3.set_ylabel("$z(t)$ [$m$]")      
ax3.grid() 
plt.show()

# gps
fig = plt.figure()
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212)

ax1.plot(np.array(data_np["t"])/1000, accel*9.81, "-k")

ax.set_xlabel("$t$ [$s$]")
ax.set_ylabel("$\\gamma(t)$ [$m/s^2$]")      
ax.grid() 
plt.show()

fig = plt.figure()
axes = fig.subplots(len(data_to_analyze_arr), 1, sharex= 'all')
fig.align_ylabels(axes)



f0=10
filter_ = False
n_points = 100
for i, ax in enumerate(axes):
    t = np.array(data_np["t"])/1000

    dat = np.array(data_np[data_to_analyze_arr[i]])
    # datnf = np.array(data[data_to_analyze_arr[i]])

    # dat = dat[0:n_points]
    # t = t[0:n_points]
    
    # if filter_ is True:
    #     for j in range(0, len(dat)-1):
    #         T = t[j+1] - t[j]
    #         tau = 1/(2*np.pi*f0)
    #         alpha = tau/T
    #         beta = 1 + alpha
    #         dat[j+1] = datnf[j]/beta + alpha/beta*dat[j]
    
    #     dat = dat[0: len(dat)-1]
    #     datnf = datnf[0: len(datnf)-1]
    #     t = t[0: len(t)-1]
        
    # data_mean = np.mean(dat)
    # data_std_dev = np.sqrt(np.var(dat))
    # data_rms = np.sqrt(np.sum((dat - data_mean)**2)/len(dat))

    # # full data correction
    # print("bias = ", data_mean)

    # print("data : " + data_to_analyze_arr[i])
    # print("Mean = " + str(data_mean))
    # print("std deviation = " + str(data_std_dev))
    # print("RMS = " + str(data_rms))
    # print("")
    
   
    ax.plot(t, dat, "b")    
    # ax.plot(t, data_mean*np.ones(len(t)), "r") 
    # ax.plot(t, (-1*data_std_dev+data_mean)*np.ones(len(t)),"--g")   
    # ax.plot(t, (1*data_std_dev+data_mean)*np.ones(len(t)),"--g")   
    
    ax.set_ylabel("$" + labels[i] + "$ [$" + units[i] + "$]")
    
    ax.tick_params(which="minor", direction="in", bottom=True, top=True, left=True, right=True)
    ax.tick_params(which="major", direction="in", bottom=True, top=True, left=True, right=True)

    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator())

    ax.get_yaxis().get_major_formatter().set_useOffset(False)
    ax.get_xaxis().get_major_formatter().set_useOffset(False)
    
    plt.subplots_adjust(wspace=0, hspace=0)
    
    if(i<len(axes)-1):
       ax.get_shared_x_axes().join(axes[i], axes[i+1])
       
    if(i == len(axes)-1):
        ax.set_xlabel("$t$ [$s$]")
        
# fig = plt.figure(dpi=300)
# axes = fig.subplots(len(data_to_analyze_arr), 1, sharex= 'all')

# f0=10
# t = np.array(data["t"])/1000

# for i, ax in enumerate(axes):
#     dat = np.array(data[data_to_analyze_arr[i]]).copy()
#     datnf = np.array(data[data_to_analyze_arr[i]]).copy()
    
#     if filter_ is True:
#         for j in range(1, len(dat)-1):
#             T = t[j] - t[j-1]
#             tau = 1/(2*np.pi*f0)
#             alpha = tau/T
#             beta = 1 + alpha
#             dat[j] = datnf[j-1]/beta + alpha/beta*dat[j-1]
        
#     freqs, psd = signal.welch(dat)
    
#     ax.semilogx(freqs, psd, "b")
    
    # gain = np.abs(fft_s[1:])*2/N
    # ESD = gain**2
    # NSD = 2*np.sum(ESD)/f0
    # print("PSD = " + str(NSD))
    
    # ax.set_ylabel(data_to_analyze_arr[i])
    
    # ax.tick_params(which="minor", direction="in",bottom=True, top=True, left=True, right=True)
    # ax.tick_params(which="major", direction="in", bottom=True, top=True, left=True, right=True)

    # y_minor = matplotlib.ticker.LogLocator(base = 10.0, subs = np.arange(1.0, 10.0) * 0.1, numticks = 10)
    
    # ax.xaxis.set_minor_locator(y_minor)
    # ax.yaxis.set_minor_locator(AutoMinorLocator())
    
    # plt.subplots_adjust(wspace=0, hspace=0)
    
    # if(i<len(axes)-1):
    #     ax.get_shared_x_axes().join(axes[i], axes[i+1])
       
    # if(i == len(axes)-1):
    #     ax.set_xlabel("t [s]")

# ax[0].legend(["Raw measurements", "mean", "1$\sigma$ interval"])
    
    # def main():
    
    
    
# if __name__ == '__main__':
#     main()

plt.show()