% Developed by Jean Gonzalez Silva 13-05-2022

% Make sure the OpenFAST directory where the FAST_SFunc.mex* file is located
% is in the MATLAB path. 
% Make sure that the matlab-toolbox is downloaded and added to the MATLAB
% path (also make sure any other OpenFAST library files that
% are needed are on the MATLAB path).
%    (relative path names are not recommended in addpath()):
% addpath('../../../build/bin'); % install location for Windows Visual Studio builds
% addpath(genpath('../../../install')); % cmake default install location

clear all; clc
global Parameters
  
% parameters required for the S-Function block:
% location of the OpenFAST input files as
OpenFASTRoot = '../../../../reg_tests/r-test/glue-codes/openfast/IEA-10-198-RWT-master/openfast/';

FAST_InputFileName = [ OpenFASTRoot 'IEA-10.0-198-RWT.fst' ];
TMax = 800;

%DTU 10MW parameters - updated to IEA 10MW
powerRated = 10e6; %[W]
%Parameters.genEfficiency = 1; % Generator efficiency
Parameters.genEfficiency = 0.94;
%Rr=89.2;

%Parameters.gbRatio=50;
Parameters.gbRatio=1;
fluidDensity=1.23; %fluidDensity=1.225;
%Jt=1.409969209e+08;
RpmtoRad=9.5493;
rpmRadSec=2*pi()/60;
Parameters.degRad=pi()/180.0;
%Parameters.bladePitchFine = 0.013090000000/Parameters.degRad; %From ROSCO [deg]
Parameters.bladePitchFine = 0.008900000000/Parameters.degRad; %From ROSCO [deg]

%Pitch control parameters
Parameters.InitializePitchControl1=true;
Parameters.InitializePitchControl2=true;
Parameters.PitchMin = Parameters.bladePitchFine; %[deg]
Parameters.PitchMax = 90; %[deg]
%PitchSwitch = 0.01745329; %[rad]
PitchSwitch = 0.017450000000; %[rad]
Parameters.PitchSwitch = PitchSwitch/Parameters.degRad; %[deg]
%load('controlTables/gainSchedulingPitch_DTU10MW_ROSCO.mat')
Parameters.gainSchedulingPitch=load('controlTables/gainSchedulingPitch_IEA10MW.mat')
Parameters.rotorSpeedInterpolant=load('controlTables/rotorSpeedInterpolant_IEA10MW_constt.mat')
Parameters.bladePitchInit=0; %From ElastoDyn (IEA-10.0)
Parameters.PitchRate=10.0; %[deg/s]

%Torque control [Based on Jonkman 2009 and ROSCO]
Parameters.VS_RtPwr = powerRated/Parameters.genEfficiency; %Rated mechanical power in Region 3 
%Parameters.KGen = 79.43986000000; % From ROSCO [Nm/((rad/s)^2)]
%Parameters.KGen = 81.1516502; % From ServoDyn (IEA-10.0) [Nm/(rpm^2)]
Parameters.KGen = 15154352.20352 % [Nm/((rad/s)^2)];
%Parameters.VS_CtInSp = 200.0/RpmtoRad; % Transitional generator speed (HSS side) bet [rad/s]
Parameters.VS_CtInSp = 200.0/RpmtoRad/50; % Transitional generator speed (HSS side) bet [rad/s] 
%Region 1.1/2 is defined to span the range of generator speed between 200
%rpm and 50% above this value (or 300rpm)
%Parameters.VS_Rgn2Sp = 300.0/RpmtoRad; %Transitional generator speed (HSS side) bet [rad/s]
Parameters.VS_Rgn2Sp = 300.0/RpmtoRad/50; %Transitional generator speed (HSS side) bet [rad/s]
VS_Rgn3MP=0.01745329; % Minimum pitch angle at which the torque is tracking [rad]
Parameters.VS_Slope15 = ( Parameters.KGen*Parameters.VS_Rgn2Sp*Parameters.VS_Rgn2Sp )/( Parameters.VS_Rgn2Sp - Parameters.VS_CtInSp );
%Parameters.VS_TrGnSp = 405.0/RpmtoRad; %[rad/s]
Parameters.VS_TrGnSp = 405.0/RpmtoRad/50; %[rad/s]
VS_SlPc = 10.0; % Rated generator slip percentage in Region 2
VS_SySp = (Parameters.VS_TrGnSp)/( 1.0 + 0.01*VS_SlPc ); %Synchronous speed of region 2 1/2 induction [rad/s]
Parameters.Region2EndGenTorque=Parameters.KGen*Parameters.VS_TrGnSp*Parameters.VS_TrGnSp;
Parameters.VS_Slope25 = Parameters.Region2EndGenTorque/( Parameters.VS_TrGnSp  - VS_SySp );
Affine_GenTorque= Parameters.Region2EndGenTorque-Parameters.VS_Slope25*Parameters.VS_TrGnSp; %from the VS_Slope25 the affine parameter with the VS_slope25
%Parameters.VS_RtGnSp = (-Affine_GenTorque+sqrt((Affine_GenTorque^2)+4*Parameters.VS_Slope25*Parameters.VS_RtPwr))/(2*Parameters.VS_Slope25); %Rated generator speed (HSS side) [rad/s] for constant power in region 3
Parameters.VS_RtGnSp = (-Affine_GenTorque+sqrt((Affine_GenTorque^2)+4*Parameters.VS_Slope25*Parameters.VS_RtPwr*0.99))/(2*Parameters.VS_Slope25); %Rated generator speed (HSS side) [rad/s] for constant torque in region 3 
PC_RefSpd = (1/0.99)*Parameters.VS_RtGnSp;
Parameters.RatedGenTorque=Parameters.VS_RtPwr/PC_RefSpd; %[Nm]
%Parameters.VS_MaxTq = 250000.0; %Maximum generator torque in Region 3 (HSS side) [Nm]
%Parameters.VS_MaxTq = 12106719.36234;
Parameters.VS_MaxTq = 12384500*1.1; % set 10% above rated
Parameters.rotorSpeedRated= PC_RefSpd/Parameters.gbRatio; %[rad/s]
Parameters.rotorSpeedInit= 7.0*rpmRadSec; %[rad/s] from ElastoDyn (IEA-10.0)
Parameters.GenTorqueRate=1.5e6;

% Filters
Parameters.F_LPFCornerFreq_rotorSpeed= 1.03398; %Corner Frequency (-3dB point) in low-pass filter of the measured rotor speed[rad/s]
%Parameters.F_LPFCornerFreq_bladePitch= 0.4; %Corner Frequency (-3dB point) in low-pass filter of the measured blade pitch signal [rad/s]
Parameters.F_LPFCornerFreq_bladePitch= 1.03398; %Corner Frequency (-3dB point) in low-pass filter of the measured blade pitch signal [rad/s]
Parameters.F_LPFCornerFreq_pitchAngleOut= 1.03398; %Corner Frequency (-3dB point) in low-pass filter of the measured rotor speed[rad/s]
Parameters.F_LPFCornerFreq_torqueOut= 0.4; %Corner Frequency (-3dB point) in low-pass filter of the output generator torque signal [rad/s]

% Reference signal
Parameters.dt = 0.0125; % From IEA-10.0-198-RWT.fst
load('AGCdata')
%initialtime=20000;
initialtime=0;
timeSetpointArray = [initialtime+Parameters.dt:Parameters.dt:initialtime+1500]; %adjusted taken into account the precursor
powerSetpointFarmArray=zeros(1,size(timeSetpointArray,2));
for ii=1:(size(timeSetpointArray,2))
if timeSetpointArray(ii)>initialtime+300
    powerSetpointFarmArray(ii) = interp1(AGCdata(:,1),AGCdata(:,2),(timeSetpointArray(ii)-initialtime+114),'linear');
end
end
%powerSetpointFarmArray=powerSetpointFarmArray.*(nTurbs*(1e6))+(nTurbs*1e6);
%powerSetpointFarmArray=powerSetpointFarmArray.*(1e6)+3e6; %set for 1 turbine
powerSetpointFarmArray=powerSetpointFarmArray.*(3e6)+3e6; %set for 1 turbine

%plot(timeSetpointArray,powerSetpointFarmArray)
%Parameters.powerDemand.time=timeSetpointArray;
Parameters.powerDemand.time=[];
Parameters.powerDemand.signals.values=powerSetpointFarmArray';
Parameters.powerDemand.signals.dimensions=1;
%Parameters.powerDemand=[timeSetpointArray' powerSetpointFarmArray'];

% run the model
sim('DR_01.mdl',[0,TMax]);

figure
plot(Time(1:size(GenPwr)),GenPwr*1000)
hold on
plot(Time(1:size(GenPwr)), OutData(:,4).*GenTorque*pi/30*Parameters.genEfficiency)
hold on
plot(Time(1:size(GenPwr)),powerSetpointFarmArray(1:size(GenPwr))','--')

figure
rotSpeedRef=Parameters.rotorSpeedInterpolant.rotorSpeedInterpolant(powerSetpointFarmArray);
plot(timeSetpointArray(1:(TMax/Parameters.dt)),rotSpeedRef(1:(TMax/Parameters.dt)))

% look at results:
% PlotFASToutput({[ OpenFASTRoot 'IEA-10.0-198-RWT.SFunc.out'],[ OpenFASTRoot 'IEA-10.0-198-RWT.out']},{'SFunc','exe'});