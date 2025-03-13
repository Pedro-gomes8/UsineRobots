% Follow trajectory robot
% Motor Parameters
Km = 50/1000 ; % 50 m*s*V'
Tau = 0.025; %sec
l = 90/1000; % meters
L = 115/1000; % meters

param = [Km Tau l L];

K_ff = 0.02;
    
Tf = 20;

px0 = 0;
py0 = 0;
theta0 = 0;

x_ref = 1;
y_ref = 1;

sim('robot.slx',Tf);

% 
% V = [v1 v2];
% U = [u1 u2];

