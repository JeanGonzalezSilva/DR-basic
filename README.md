Created by: Jean Gonzalez Silva
Date: 10/06/2022

Make sure you have the input files from the IEA 10MW RWT and the path is appropriatly set in Run_down_regulator.m.
They can be found in https://github.com/IEAWindTask37/IEA-10.0-198-RWT

%% Contents ~ Meanings:

Run_down_regulator ~ MATLAB script to initialize parameters and run the Simulink model.

DR_01 ~ Simulink model.

DR_PowerSpeedCurve ~ MATLAB function with the down-regulator code.
Two distinct down-regulation modes are implemented to reduce the undesirable jumps when the power reference overcome the available power.

AGCdata ~ Data for the input power reference.

controlTables ~ Includes the lookup tables for the gain-scheduled blade pitch controller and the rotor speed reference.
In addition, it includes the codes to generate such tables.

results ~ Preliminar results on the implemented 2 derating strategies 
