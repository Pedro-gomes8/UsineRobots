% Follow trajectory robot
% Motor Parameters
Km = 50/1000 ; % 50 m*s*V'
Tau = 0.025; %sec
R = 0.17; % [m]
V_max = 0.7; % m/s
W_max = pi; % rad/s

param = [Km Tau R];

K_ff = 0.02;

u_sat = 0.5*V_max;

pre_gain_v = 1/0.8;
pre_gain_w = 1/0.9;

% ===========================
px0 = 0;
py0 = 0;
theta0 = 0;

x_ref = 10;
y_ref = 15;


% ==========================

Tf = 100;
sim('robot.slx',Tf);

% 
% V = [v1 v2];
% U = [u1 u2];

