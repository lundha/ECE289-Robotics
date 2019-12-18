%% Graduate project - Martin Haug

% Initalize 
run graduate_proj_eom.m

% Print system matrices
A, B, C, D

% Simulation and animation of open loop dynamics
run Problem2.m

input('Press enter to continue');

% Animation and plotting of closed loop system initialized
% close to equilibrium point
run Problem3_d.m

input('Press enter to continue');

% Animation and plotting of closed loop system initialized
% far from equilibrium point
run Problem3_e.m



