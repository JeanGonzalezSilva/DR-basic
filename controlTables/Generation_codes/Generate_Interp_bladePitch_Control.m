% Created by Jean Gonzalez Silva - 13/05/2022
% This script creates a gridded data set to perform interpolation from the
% OpenFAST input file - ServoDyn.
% Make sure that the matlab-toolbox is downloaded and added to the MATLAB path

DataOut=FAST2Matlab([OpenFASTRoot 'IEA-10.0-198-RWT_DISCON.IN'])

n=DataOut.Val{21}; %Amount of gain-scheduling table entries
pitch = DataOut.Val{22};
Kp = DataOut.Val{23};
Ki = DataOut.Val{24};
Kd = DataOut.Val{25};


gainSchedulingPitch.Kp=griddedInterpolant(pitch,Kp);
gainSchedulingPitch.Ki=griddedInterpolant(pitch,Ki);
gainSchedulingPitch.Kd=griddedInterpolant(pitch,Kd);
save('gainSchedulingPitch.mat','-struct','gainSchedulingPitch')


















