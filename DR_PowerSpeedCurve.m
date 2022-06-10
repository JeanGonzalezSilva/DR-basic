function uout = DR_PowerSpeedCurve(uin)
global Parameters
persistent intSpeedError speedErrorLast genTorqueLast bladePitchLast rotorSpeedFiltered bladePitchFiltered torqueOutFiltered pitchAngleOutFiltered iter

powerSetpoint=uin(1);
rotorSpeed=uin(2);
bladePitch=uin(3);

downRegulationMode=2;  % Type of down-regulation {1=near-perfect tracking 
% % leaded by the generator torque [peak issues when saturation occurs],
% 2=lead by pitch with minimum torque strategy [smoother transitions but
% delay on tracking]

% Initializing controllers and filters
if isempty(iter) 
iter = 1;
intSpeedError =0;
speedErrorLast =0;
powerSetpoint=Parameters.powerDemand.signals.values(1);
rotorSpeed=Parameters.rotorSpeedInit;
bladePitch=Parameters.bladePitchInit;
rotorSpeedFiltered=rotorSpeed;
bladePitchFiltered=bladePitch;
genTorqueLast =powerSetpoint/rotorSpeed/Parameters.genEfficiency;
bladePitchLast=Parameters.bladePitchFine;
torqueOutFiltered=genTorqueLast;
pitchAngleOutFiltered=bladePitchLast;
end

% Set the rotor speed reference from a look-up table based on the 
% conventional torque law in below rated conditions
rotorSpeedReference = Parameters.rotorSpeedInterpolant.rotorSpeedInterpolant(powerSetpoint); %[rad/s]  

% Filter rotor speed measurements
betaf1=exp(-Parameters.F_LPFCornerFreq_rotorSpeed*Parameters.dt); % betaf=exp(-wo*T), T=0.0125 & wo(-3dB)=1.12975 rad/s -> betaf1=0.986 (ROSCO)
rotorSpeedFiltered= rotorSpeedFiltered*betaf1 + (1-betaf1)*rotorSpeed;

% Filter blade pitch measurements
betaf2=exp(-Parameters.F_LPFCornerFreq_bladePitch*Parameters.dt); % betaf=exp(-wo*T), T=0.0125 & wo(-3dB)=0.4 rad/s -> betaf2=0.995
bladePitchFiltered= bladePitchFiltered*betaf2 + (1-betaf2)*bladePitch;

if (powerSetpoint>Parameters.VS_RtPwr) || (rotorSpeed<rotorSpeedReference && bladePitch<=Parameters.bladePitchFine) %TEST - COND 1
%if (powerSetpoint>Parameters.VS_RtPwr) || (rotorSpeedFiltered<rotorSpeedReference && bladePitchFiltered<=Parameters.bladePitchFine) %TEST - COND 2
%if (powerSetpoint>Parameters.VS_RtPwr) || (rotorSpeedFiltered<rotorSpeedReference && bladePitchFiltered<Parameters.PitchSwitch) %TEST - COND 3
            turbIsGreedy = true;
			disp(['Turbine is greedy.'])
			%TORQUE CONTROLLER
            genSpeed = rotorSpeedFiltered * Parameters.gbRatio; % in rad/s
            torqueGreedy   = Parameters.KGen * (genSpeed.^2); % Greedy control signal (Does Kgen is set fot genSpeed or rotSpeed?)
            torqueTracking =  Parameters.RatedGenTorque; % Constant torque [also need to change the transition
                if (genSpeed > Parameters.VS_RtGnSp)
                    disp(['Current torque control mode: Region 3.']);
                    torqueOut = torqueTracking;
                elseif genSpeed< (Parameters.VS_CtInSp)
                    torqueOut=0;
                    disp(['Current torque control mode: Region 1.']);
                elseif genSpeed< (Parameters.VS_Rgn2Sp)   
                    torqueOut= Parameters.VS_Slope15*( genSpeed - Parameters.VS_CtInSp);
                    disp(['Current torque control mode: Region 1.1/2.']);
                elseif genSpeed< (Parameters.VS_TrGnSp)
                    disp(['Current torque control mode: Region 2.']);
                    torqueOut = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
                else
                    disp(['Current torque control mode: Region 2.1/2.']);
                    torqueOut= Parameters.Region2EndGenTorque + Parameters.VS_Slope25*( genSpeed - Parameters.VS_TrGnSp);
                end
       
				%BLADE PITCH CONTROLLER - Gain-scheduling PID CONTROL
                if rotorSpeedFiltered<Parameters.rotorSpeedRated &&  bladePitchFiltered<Parameters.PitchSwitch
                    pitchAngleOut=Parameters.bladePitchFine;
					Parameters.InitializePitchControl1=true;
                else
					if Parameters.InitializePitchControl1
							PitchControlKI = -Parameters.gainSchedulingPitch.Ki(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
							intSpeedError= (bladePitchFiltered*Parameters.degRad) / PitchControlKI;
							speedErrorLast= 0;
						Parameters.InitializePitchControl1=false;
					end
				%Compute the gains
				PitchControlKP= -Parameters.gainSchedulingPitch.Kp(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
				PitchControlKI= -Parameters.gainSchedulingPitch.Ki(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
				PitchControlKD= -Parameters.gainSchedulingPitch.Kd(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
				%Compute the low speed shaft speed error.
				speedError= rotorSpeedFiltered- Parameters.rotorSpeedRated; %[rad/s]
				%Numerically integrate the speed error over time.
				intSpeedError= intSpeedError+speedError*Parameters.dt;
				%Numerically take the deriviative of speed error w.r.t time.
				derivSpeedError = (speedError - speedErrorLast) / Parameters.dt;
				%Store the old value of speed error.
				speedErrorLast=speedError;
				%Saturate the integrated speed error based on pitch saturation.
				intSpeedError = min(max(intSpeedError, Parameters.PitchMin*Parameters.degRad/PitchControlKI), Parameters.PitchMax*Parameters.degRad/PitchControlKI);
				%Compute the pitch components from the proportional, integral, and derivative parts and sum them.
				pitchP =  PitchControlKP* speedError; %[rad]
				pitchI =  PitchControlKI* intSpeedError; %[rad]
				pitchD =  PitchControlKD* derivSpeedError; %[rad]
				pitchAngleOut= (pitchP + pitchI + pitchD) / Parameters.degRad; %[deg]
				pitchAngleOut= min(max(pitchAngleOut, Parameters.PitchMin), Parameters.PitchMax);
                end
                Parameters.InitializePitchControl2=true;
        else
          %POWER TRACKING ALGORITHM (based on KNU2 and pitch reserve method) 
			turbIsGreedy = false;
			disp(['Turbine is tracking power.'])
			%PITCH POWER REFERENCE CONTROL
                rotorSpeedReference = min(rotorSpeedReference,Parameters.rotorSpeedRated);
					if Parameters.InitializePitchControl2   
							PitchControlKI = -Parameters.gainSchedulingPitch.Ki(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
							intSpeedError = (bladePitchFiltered*Parameters.degRad) / PitchControlKI;
							speedErrorLast= 0;
						Parameters.InitializePitchControl2=false;
					end
				%Compute the gains
				PitchControlKP= -Parameters.gainSchedulingPitch.Kp(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
				PitchControlKI= -Parameters.gainSchedulingPitch.Ki(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
				PitchControlKD= -Parameters.gainSchedulingPitch.Kd(bladePitchFiltered*Parameters.degRad)*Parameters.gbRatio;
				%Compute the low speed shaft speed error.
				speedError= rotorSpeedFiltered- rotorSpeedReference; %[rad/s]
				%Numerically integrate the speed error over time.
				intSpeedError= intSpeedError+speedError*Parameters.dt;
				%Numerically take the deriviative of speed error w.r.t time.
				derivSpeedError = (speedError - speedErrorLast) / Parameters.dt;
				%Store the old value of speed error.
				speedErrorLast=speedError;
				%Saturate the integrated speed error based on pitch saturation.
				intSpeedError = min(max(intSpeedError, Parameters.PitchMin*Parameters.degRad/PitchControlKI), Parameters.PitchMax*Parameters.degRad/PitchControlKI);
				%Compute the pitch components from the proportional, integral, and derivative parts and sum them.
				pitchP =  PitchControlKP* speedError; %[rad]
				pitchI =  PitchControlKI* intSpeedError; %[rad]
				pitchD =  PitchControlKD* derivSpeedError; %[rad]
				pitchAngleOut= (pitchP + pitchI + pitchD) / Parameters.degRad; %[deg]
				pitchAngleOut= min(max(pitchAngleOut, Parameters.PitchMin), Parameters.PitchMax);       
			
			%TORQUE CONTROLLER
			genSpeed = rotorSpeedFiltered * Parameters.gbRatio; % in rad/s
            torqueTracking =  powerSetpoint / (genSpeed*Parameters.genEfficiency); % Tracking control signal 
            if downRegulationMode==1
                torqueOut=torqueTracking;
            elseif downRegulationMode==2
                torqueGreedy   = Parameters.KGen * (genSpeed.^2); % Greedy control signal (Does Kgen is set fot genSpeed or rotSpeed?  
                if (genSpeed > Parameters.VS_RtGnSp)
                    disp(['Current torque control mode: Region 3.']);
                    torqueOutConv = Parameters.RatedGenTorque; % Constant torque [also need to change the transition]
                elseif genSpeed< (Parameters.VS_CtInSp)
                    torqueOutConv=0;
                    disp(['Current torque control mode: Region 1.']);
                elseif genSpeed< (Parameters.VS_Rgn2Sp)   
                    torqueOutConv= Parameters.VS_Slope15*( genSpeed - Parameters.VS_CtInSp);
                    disp(['Current torque control mode: Region 1.1/2.']);
                elseif genSpeed< (Parameters.VS_TrGnSp)
                    disp(['Current torque control mode: Region 2.']);
                    torqueOutConv = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
                else
                    disp(['Current torque control mode: Region 2.1/2.']);
                    torqueOutConv= Parameters.Region2EndGenTorque + Parameters.VS_Slope25*( genSpeed - Parameters.VS_TrGnSp);
                end
                torqueOut = min( torqueTracking, torqueOutConv); %see more details in Silva 2022 at TORQUE paper
                if torqueOut==torqueTracking
                    disp(['Torque tracking has been applied']);
                else
                    disp(['Torque greedy has been applied']);
                end
            else
                disp(['The down-regulation mode is not found.']);
                return
            end
            Parameters.InitializePitchControl1=true;
        end

        % Saturate using the maximum torque limit
        if (min(torqueOut,Parameters.VS_MaxTq)==Parameters.VS_MaxTq)
            torqueOut=Parameters.VS_MaxTq;
            disp(['Generator torque reaches maximum!']);
        end 

        % Generator torque command filter
        %{
        betaf3=exp(-Parameters.F_LPFCornerFreq_torqueOut*Parameters.dt); % betaf=exp(-wo*T), T=0.0125 & wo(-3dB)=0.4 rad/s -> betaf = 0.9950
        torqueOutFiltered= torqueOutFiltered*betaf3 + (1-betaf3)*torqueOut;
        torqueOut=torqueOutFiltered;
        %}
				
        % Blade pitch command filter
        %{
        betaf4=exp(-Parameters.F_LPFCornerFreq_pitchAngleOut*Parameters.dt); 
        pitchAngleOutFiltered= pitchAngleOutFiltered*betaf4 + (1-betaf4)*pitchAngleOut;
        pitchAngleOut=pitchAngleOutFiltered;
        %}

        % RATE LIMITERS
        applyRateLimiterTorque = true;
        if applyRateLimiterTorque
            torqueRateLimit = Parameters.GenTorqueRate * Parameters.dt;
            deltaTorque = torqueOut - genTorqueLast;
            deltaTorque = max(min(deltaTorque,torqueRateLimit),-torqueRateLimit);
            torqueOut = genTorqueLast + deltaTorque;
        end
        applyRateLimiterPitch = true;
        if applyRateLimiterPitch
            pitchRateLimit = Parameters.PitchRate * Parameters.dt;
            deltaPitch = pitchAngleOut - bladePitchLast;
            deltaPitch = max(min(deltaPitch,pitchRateLimit),-pitchRateLimit);
            pitchAngleOut = bladePitchLast + deltaPitch;
        end
        genTorqueLast=torqueOut;
        bladePitchLast=pitchAngleOut;
uout=[torqueOut; pitchAngleOut];
iter = iter + 1;
end
