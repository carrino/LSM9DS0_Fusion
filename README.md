# LSM9DS0_Fusion

This code is designed to take the input from a LSM9DS0 9DOF device and convert it to an orientation.

It uses the accelerometer to determine which way is down and the compass for north. 

Gyro updates are applied right away and the other 2 sensors are slowly added in to correct for gyro drift.

This depends on https://github.com/carrino/Quaternion and https://github.com/adafruit/Adafruit_LSM9DS0_Library

# LSM9DS1
Support added by Enrique Rodriguez on branch LSM9DS1: https://github.com/carrino/LSM9DS0_Fusion/tree/LSM9DS1
