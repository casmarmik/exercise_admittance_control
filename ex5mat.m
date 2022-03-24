clc;
clear;
close all;
%% ex5mat
model = importrobot("ur5e_3link.urdf");
model.DataFormat = 'column';
model.Gravity = [0, 0, -9.82]';

showdetails(model)

or = [0.5 0 0];
K = [10 0 0; 0 0 0; 0 0 0];
D = [4 0 0 ; 0 1 0; 0 0 1];
Kp = [2 0 0; 0 1 0; 0 0 2];
Mx = [4/3 0 0; 0 1/12 0; 0 0 1/16];
fx = [1 0 0; 0 2 0; 0 0 3];
D = 2 * sqrt((fx+Kp)*Mx);
x = D/(2*sqrt(Mx*(Kp+fx)));
Minv = inv(Mx); 
pd = [0.7 0 0];
time = [0:0.01:25];
timeforce = [0:0.01:5];
movex = sin(time);
movey = sin(pi/2*time);
movez = sin(pi/3*time);
%force = time;
%force(1:501) = zeros(1,length(timeforce))';
forcex = timeforce;
forcey = timeforce;
forcez = timeforce;
forcex(1:end) = 1;
forcey(1:end) = 2;
forcez(1:end) = 3;
%move(1:101) = 0:0.01:1;
%move(102:length(move)) = 1;
force = [time; zeros(1,length(timeforce)-1) forcex zeros(1,length(timeforce)*3-3); zeros(1,length(timeforce)-1) forcey zeros(1,length(timeforce)*3-3); zeros(1,length(timeforce)-1) forcez zeros(1,length(timeforce)*3-3)]';
motion = [time; movex; movey; movez]';
%D = 2 * sqrt((fx+Kp)*Mx)
Kf = [0.075 0 0; 0 1 0; 0 0 1];
Ki = [0.01 0 0; 0 1 0; 0 0 1];
fd = [5 0 0];

%% Orientational

Ko = [2 0 0; 0 1 0; 0 0 2];
Mo = [4/3 0 0; 0 1/12 0; 0 0 1/16];
mud = [1 0 0; 0 0.5 0; 0 0 1];
Do = 2 * sqrt((mud+Ko)*Mo);
xo = Do/(2*sqrt(Mo*(Ko+mud)));
Moinv = inv(Mo); 

torquex = timeforce;
torquey = timeforce;
torquez = timeforce;
torquex(1:end) = mud(1,1);
torquey(1:end) = mud(2,2);
torquez(1:end) = mud(3,3);
torque = [time; zeros(1,length(timeforce)*3-3) torquex zeros(1,length(timeforce)-1); zeros(1,length(timeforce)*3-3) torquey zeros(1,length(timeforce)-1); zeros(1,length(timeforce)*3-3) torquez zeros(1,length(timeforce)-1)]';

movexo = sin(time);
moveyo = sin(pi/2*time);
movezo = sin(pi/3*time);
motiono = [time; movexo; moveyo; movezo]';

dt = 0.01;




%% hybrid
Sf = [1 0 0];
Sv = [0 1 0; 0 0 1]';
Sfinv = pinv(Sf);
Svinv = pinv(Sv);
