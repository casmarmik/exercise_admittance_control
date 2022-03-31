%% Euler Angle based controller

clear
clc

dt = 0.001; %Timestep
time = 0:dt:25; % Time the controller is be simulated in
torqueTime = 0:dt:5; % Time the torque is applied

% Defining the torques applied
torqueX = zeros(1,5/dt+1) + 1;
torqueY = zeros(1,5/dt+1) + 0.5;
torqueZ = zeros(1,5/dt+1) + 1;

% Combining the data into a matrix corresponding to the information given
% in the hand-in description
% To use in Simulink, the matrix is read row-wise with the first column
% specifying the the time and the subsequent columns specifying data values
torqueProfile = [time; 
                zeros(1,length(torqueTime)*3-3) torqueX zeros(1,length(torqueTime)-1); 
                zeros(1,length(torqueTime)*3-3) torqueY zeros(1,length(torqueTime)-1); 
                zeros(1,length(torqueTime)*3-3) torqueZ zeros(1,length(torqueTime)-1)]';

% Specifying a motion for the joint angles to use for the simulation
moveXE = sin(time);
moveYE = sin(pi/2*time);
moveZE = sin(pi/3*time);
motionProfile = [time; moveXE; moveYE; moveZE]';

% Gain matricies. These are chosen at random
M0 = [2 0 0;
      0 1 0;
      0 0 2];
K0 = [4 0 0;
      0 5 0;
      0 0 5];

% Defining enviromental stifness. This is what we aim for the controller to
% be critically damped towards
K = [1 0 0; 0 0.5 0; 0 0 1];

% Calculating D0 based on previously defined matricies and critically
% damped system, xi = 1 (See lecture 5, slide 34)
D0 = 2 * sqrt(M0*(K0));
M0inv = inv(M0);