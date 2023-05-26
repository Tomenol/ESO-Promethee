# ESO-Promethee

## Project description
ESO-Prométhée was an experimental rocket project which was designed and built by the Estaca Space Odyssey student association in 2022 at the C'Space student launch campain organized by CNES and Planète Science. 

![alt text](https://eso-estaca.fr/wp-content/uploads/photo-gallery/cspace2022-fx28-8.jpg?bwg=1662997060)
__Credits:__ https://eso-estaca.fr/

The objective of this experimental rocket was to test a Low-Cost navigation software (based on an Extended Kalman filter EKF) developed by the team responsible for the rocket's experimental payload. 

## Repository
This repository contains the four main Python/C/C++ modules implemented during the project:
- The *Embedded flight software* designed to run on two separate boards: MASTER and SLAVE. The role of the MASTER board is to read and process the measurements from the pressure sensors, IMUs, magnetometers and GPS boards. On the other hand, the SLAVE board stores the processed data transmitted by the MASTER board on an SD card.
- The *Rocket flight simulation software*, used to simulate the full rocket flight dynamics (Models: wind, aerodynamic, propulsion, ...) and generate the corresponding sensor data (Models: magnetic, atmospheric, dynamics, ...). This simulator was then used to design and tune the EKF navigation algorithm on realistic flight data.
- The *Navigation system* EKF-based used to retrieve the rocket trajectory developed in python.  
- The *Data analysis scripts* used to process flight data. Due to the failure of the GPS module, the full EKF could not be successfully applied to retrieve the rocket trajectory. However, the flight data was used to test the apogee predictions of the flight simulation software.

## Results
### Flight simulation software
The following results correpond to the predicted trajectory of the rocket for average wind conditions and assuming nominal mass, dimensions, propulsion performances and atmosphere.

#### Rocket position exrpessed in the ENU coordinate frame
![states_p_enu](https://github.com/Tomenol/ESO-Promethee/assets/54234406/d3772454-75fc-4cbd-b17c-592d8ded7c4a)

#### Angular velocities expressed in the ENU coordinate frame
![states_omega_enu](https://github.com/Tomenol/ESO-Promethee/assets/54234406/6179e81a-56b8-4713-9a8c-381bb36be090)

#### Quaternion vector
![states_q](https://github.com/Tomenol/ESO-Promethee/assets/54234406/9b20aec7-2b29-4ac0-b2c3-3a5b0d0b83dc)

#### Velocity in the ENU reference frame
![states_v_enu](https://github.com/Tomenol/ESO-Promethee/assets/54234406/af72516f-c144-4770-a870-729d932d6640)

### Tuning of the Extended Kalman Filter (attitude determination)
The following results correpond to the rocket attitude predicted by the navigation software (in blue) using simulated flight data (in red).

#### Simulated and predicted quaternion vector
![kalman_quaternion_q](https://github.com/Tomenol/ESO-Promethee/assets/54234406/b33922ae-d5d0-44bb-8b12-f8004de1a809)

### Real flight data
The following results showcase the processed flight data and the corresponding altitude prediction.

#### Measured Pressure and Temperature
![data_fusex_PT](https://github.com/Tomenol/ESO-Promethee/assets/54234406/86d39eae-c665-4648-ac34-790585a34e17)

#### Measured acceleration in the rocket frame
![data_fusex_accel_lo](https://github.com/Tomenol/ESO-Promethee/assets/54234406/3128346c-6277-48a1-b53a-e2f13e24d2dc)

#### Measured angular velocity in the rocket frame
![data_fusex_omega_lo](https://github.com/Tomenol/ESO-Promethee/assets/54234406/075a1d48-1814-45b3-bc5c-eb5988ba82c6)

#### Predicted altitude using accelerometer/gyrometer integrated measurements, pressure/temperature and the EKF 
![data_fusex_z_ekf](https://github.com/Tomenol/ESO-Promethee/assets/54234406/94ca55cf-5f97-4bae-84d3-1439f99e26a6)

The altitude predicted by the EKF is very similar to the one found with the full flight simulation (take-off @ t=5s):
![image](https://github.com/Tomenol/ESO-Promethee/assets/54234406/fc5f9f08-bc1b-490c-b199-2de8b959d42f)

