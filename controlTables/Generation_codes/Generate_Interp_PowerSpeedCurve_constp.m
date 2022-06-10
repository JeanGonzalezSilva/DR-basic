%% Generate Lookup table (power versus generator speed)
% The table is based on the generator torque for the greedy operation
% Developed at 12-08-2021 by Jean Gonzalez Silva
clear all
%DataOut=FAST2Matlab([OpenFASTRoot 'IEA-10.0-198-RWT_DISCON.IN'])
global Parameters

% Loop initialization
firstRun = true;
powerRated = 10e6; %Watts
Parameters.genEfficiency = 0.94;
Parameters.gbRatio=1;
Parameters.degRad=pi()/180.0;
Parameters.bladePitchFine = 0.008900000000/Parameters.degRad; %From ROSCO [deg]

Rr=198.0/2;
RpmtoRad=9.5493;
rpmRadSec=2*pi()/60;

torqueArrayOut = [];
powerArrayOut = [];
generatorSpeedArrayOut = [];
pitchAngleArrayOut = Parameters.bladePitchFine;

powerArrayOutTable = [];
generatorSpeedArrayOutTable = [];
%load('cpInterpolant.mat')

%Torque control [Based on Jonkman 2009 and ROSCO]
Parameters.VS_RtPwr = powerRated/Parameters.genEfficiency; %Rated mechanical power in Region 3 [???]
%Parameters.KGen = 79.43986000000; % From ROSCO [Nm/((rad/s)^2)]
%Parameters.KGen = 81.1516502; % From ServoDyn (IEA-10.0)
Parameters.KGen = 15154352.20352 % [Nm/((rad/s)^2)];
%Parameters.VS_CtInSp = 200.0/RpmtoRad; % Transitional generator speed (HSS side) bet [rad/s] [??? dividir by the old gbRatio= 50????]
Parameters.VS_CtInSp = 200.0/RpmtoRad/50; % Transitional generator speed (HSS side) bet [rad/s] [??? dividir by the old gbRatio= 50????]
%Region 1.1/2 is defined to span the range of generator speed between 200
%rpm and 50% above this value (or 300rpm)
%Parameters.VS_Rgn2Sp = 300.0/RpmtoRad; %Transitional generator speed (HSS side) bet [rad/s]
Parameters.VS_Rgn2Sp = 300.0/RpmtoRad/50; %Transitional generator speed (HSS side) bet [rad/s]
%VS_Rgn3MP=0.01745329; % Minimum pitch angle at which the torque is tracking [rad]
Parameters.VS_Slope15 = ( Parameters.KGen*Parameters.VS_Rgn2Sp*Parameters.VS_Rgn2Sp )/( Parameters.VS_Rgn2Sp - Parameters.VS_CtInSp );
%Parameters.VS_TrGnSp = 405.0/RpmtoRad; %[rad/s]
Parameters.VS_TrGnSp = 405.0/RpmtoRad/50; %[rad/s]
VS_SlPc = 10.0; % Rated generator slip percentage in Region 2
VS_SySp = (Parameters.VS_TrGnSp)/( 1.0 + 0.01*VS_SlPc ); %Synchronous speed of region 2 1/2 induction [rad/s]
Parameters.Region2EndGenTorque=Parameters.KGen*Parameters.VS_TrGnSp*Parameters.VS_TrGnSp;
Parameters.VS_Slope25 = Parameters.Region2EndGenTorque/( Parameters.VS_TrGnSp  - VS_SySp );
Affine_GenTorque= Parameters.Region2EndGenTorque-Parameters.VS_Slope25*Parameters.VS_TrGnSp; %from the VS_Slope25 the affine parameter with the VS_slope25
%Parameters.VS_RtGnSp = (-Affine_GenTorque+sqrt((Affine_GenTorque^2)+4*Parameters.VS_Slope25*0.95*Parameters.VS_RtPwr))/(2*Parameters.VS_Slope25); %Rated generator speed (HSS side) [rad/s]
Parameters.VS_RtGnSp = (-Affine_GenTorque+sqrt((Affine_GenTorque^2)+4*Parameters.VS_Slope25*Parameters.VS_RtPwr))/(2*Parameters.VS_Slope25); %Rated generator speed (HSS side) [rad/s]
PC_RefSpd = (1/0.95)*Parameters.VS_RtGnSp;
Parameters.RatedGenTorque=Parameters.VS_RtPwr/PC_RefSpd; %[Nm]
%Parameters.VS_MaxTq = 250000.0; %Maximum generator torque in Region 3 (HSS side) [Nm]
Parameters.VS_MaxTq = 12106719.36234; %??
Parameters.rotorSpeedRated= PC_RefSpd/Parameters.gbRatio; %[rad/s]
Parameters.rotorSpeedInit= 7.0*rpmRadSec; %[rad/s] from ElastoDyn (IEA-10.0)

rotorSpeedRated_fromProp=9.6/RpmtoRad; % in rad/s

torqueTracking=Parameters.VS_RtPwr/Parameters.VS_RtGnSp;

for genSpeed=0:0.001:(PC_RefSpd)
         % TORQUE CONTROLLER
        torqueGreedy   = Parameters.KGen * (genSpeed.^2); % Greedy control signal
        if (genSpeed > Parameters.VS_RtGnSp)
            disp(['Current torque control mode: Region 3.']);
            torque = Parameters.VS_RtPwr/genSpeed; % Force perfect tracking
            %torque = RatedGenTorque; %Constant torque
        elseif genSpeed < (Parameters.VS_CtInSp)
            torque=0;
            disp(['Current torque control mode: Region 1.']);
        elseif genSpeed< (Parameters.VS_Rgn2Sp)   
            torque= Parameters.VS_Slope15*( genSpeed - Parameters.VS_CtInSp);
            disp(['Current torque control mode: Region 1.1/2.']);
        elseif genSpeed< (Parameters.VS_TrGnSp)
            disp(['Current torque control mode: Region 2.']);
            torque = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
        else
            disp(['Current torque control mode: Region 2.1/2.']);
            torque= Parameters.Region2EndGenTorque + Parameters.VS_Slope25*( genSpeed - Parameters.VS_TrGnSp);
        end
        
        if genSpeed*torque*Parameters.genEfficiency>0 && genSpeed*torque*Parameters.genEfficiency~=Parameters.VS_RtPwr
        generatorSpeedArrayOut =[generatorSpeedArrayOut; genSpeed];  % in rad/s
        torqueArrayOut =[torqueArrayOut; torque]; % in N.m
        powerArrayOut = [powerArrayOut; genSpeed*torque*Parameters.genEfficiency]; % in W 
        
        if genSpeed<=Parameters.VS_RtGnSp %Only below rated conditions
            powerArrayOutTable=[powerArrayOutTable; genSpeed*torque*Parameters.genEfficiency];
            generatorSpeedArrayOutTable =[generatorSpeedArrayOutTable; genSpeed];  % in rad/s
        end
        end
end

%%{
figure
plot(generatorSpeedArrayOut, torqueArrayOut)
ylabel('Torque Command [Nm]')
xlabel('Rotor Speed [rad/s]')
grid on 
%ylim([20,50])
%legend('Classical Controller','RRSR','RTSRR')

%export_fig 'Lookup_Table_P_omega_DTU10MW_ROSCO_constant_power.pdf' -transparent -nocrop% -painters'

%{
figure
plot(powerArrayOutTable, (RpmtoRad)*generatorSpeedArrayOutTable/Parameters.gbRatio)
xlabel('Power Command [W]')
ylabel('Rotor Speed [rpm]')
%}

figure
plot(powerArrayOutTable, generatorSpeedArrayOutTable/Parameters.gbRatio)
xlabel('Power Command [W]')
ylabel('Rotor Speed [rad/s]')
%}

%%{
rotorSpeedArrayOutTable=generatorSpeedArrayOutTable/Parameters.gbRatio; % in rad/s
rotorSpeedInterpolant=griddedInterpolant(powerArrayOutTable,rotorSpeedArrayOutTable,'linear','linear');
%save('rotorSpeedInterpolant.mat','rotorSpeedInterpolant')

%rotorSpeedInterpolant_DTU10MW(10000000)

%}
