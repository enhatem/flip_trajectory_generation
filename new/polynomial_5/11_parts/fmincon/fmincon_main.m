clear; close all; clc;

%% Drone parameters and costants

global g t_step z1_min z2_min z3_min z1_max z2_max z3_max zd_min zd_max zdd_min zdd_max m Ixx l u1_max u2_max

% Drone parameters
m =  29e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
t_step = 0.01; % 100 Hz
g = 9.81; % m/s^2

%% Constaints on z that are applied on the optimization problem
% Bounds on the z trajectory of the reaching phase
z1_min = 0.8;
z1_max = 2;

% Bounds on the z trajectory of the flipping phase
z2_min = 0.8;
z2_max = 2;

% Bounds on the z trajectory of the recovery phase
z3_min = 0.8;
z3_max = 2;

%% Constraints on the Thrust and Torques that are applied on the opitmization problem
% Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 46e-3 * g ) / 2 ); % Maximum thrust 
u2_max = 0.1 * ( 1 / 2 * u1_max * l); % Maximum torque

%% Constraints on phi that are applied on the optimization problem

% lower bound on the initial phi angle at the start of the reaching phase
phi_start_min = 0;

% upper bound on the final phi angle at the end of the reaching phase
phi_start_max = pi/2;

% lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_end_min = 3/2*pi;

% upper bound on the final phi angle at the end of the recovery phase
phi_end_max   = 2*pi;


%% Constraints on the time that are applied on the optimization problem

% bounds on t1 (time of the first trajectory in the reaching phase)
t1_min = 0.1;
t1_max = inf;

% bounds on t2 (time of the second trajectory in the reaching phase)
t2_min = 0.1;
t2_max = inf;

% bounds on t3 (time of the third trajectory in the reaching phase)
t3_min = 0.1;
t3_max = inf;

% bounds on t4 (time of the fourth trajectory in the reaching phase)
t4_min = 0.1;
t4_max = inf;

% bounds on t5 (time of the fifth (final) trajectory in the reaching phase)
t5_min = 0.1;
t5_max = inf;

% bounds on t6 (time of the flip phase trajectory)
t6_min = 0.1;
t6_max = inf;

% bounds on t7 (time of the first trajectory in the recovery phase)
t7_min = 0.1;
t7_max = inf;

% bounds on t8 (time of the second trajectory in the recovery phase)
t8_min = 0.1;
t8_max = inf;

% bounds on t9 (time of the third trajectory in the recovery phase)
t9_min = 0.1;
t9_max = inf;

% bounds on t10 (time of the fourth trajectory in the recovery phase)
t10_min = 0.1;
t10_max = inf;

% bounds on t11 (time of the fifth trajectory in the recovery phase)
t11_min = 0.1;
t11_max = inf;

%% Constraints on zd that are applied to the optimization problem
zd_min = -inf;
zd_max = inf;

%% Constraints on zdd that are applied to the optimization problem
zdd_min = -inf;
zdd_max =  inf;

%% Constraint of phid that are applied to the optimization problem
phid_min = -inf;
phid_max =  inf;

%% Constraints of phidd that are applied to the optimization problem
phidd_min = -inf;
phidd_max =  inf;

% lower and upper bounds

lb = [z1_min z1_min z1_min z1_min z1_min z1_min z2_min z3_min z3_min z3_min z3_min z3_min zd_min zd_min zd_min zd_min zd_min zd_min zd_min zd_min zdd_min zdd_min zdd_min zdd_min zdd_min zdd_min zdd_min zdd_min phi_start_min phi_start_min phi_start_min phi_start_min phi_start_min phi_end_min phi_end_min phi_end_min phi_end_min phi_end_min phid_min phid_min phid_min phid_min phid_min phid_min phid_min phid_min phidd_min phidd_min phidd_min phidd_min phidd_min phidd_min phidd_min phidd_min t1_min t2_min t3_min t4_min t5_min t6_min t7_min t8_min t9_min t10_min t11_min];
ub = [z1_max z1_max z1_max z1_max z1_max z1_max z2_max z3_max z3_max z3_max z3_max z3_max zd_max zd_max zd_max zd_max zd_max zd_max zd_max zd_max zdd_max zdd_max zdd_max zdd_max zdd_max zdd_max zdd_max zdd_max phi_start_max phi_start_max phi_start_max phi_start_max phi_start_max phi_end_max phi_end_max phi_end_max phi_end_max phi_end_max phid_max phid_max phid_max phid_max phid_max phid_max phid_max phid_max phidd_max phidd_max phidd_max phidd_max phidd_max phidd_max phidd_max phidd_max t1_max t2_max t3_max t4_max t5_max t6_max t7_max t8_max t9_max t10_max t11_max];

%%

% nonlinear bounds
nl_con = @nonlinear_bounds;

% objective function
obj = @objective_function;

%% Optimization problem

% Initial condition
% For z
z1  = 1.0; % Initial position (start of reaching phase)
z2  = 1.1; % First intermediate waypoint(reaching phase)
z3  = 1.2; % Second intermediate waypoint(reaching phase)
z4  = 1.3; % Third intermediate waypoint(reaching phase)
z5  = 1.4; % Fourth intermediate waypoint(reaching phase)
z6  = 1.5; % Final position of the recovery phase (start of the flip phase)
z7  = 1.4; % Final position of the flip phase (start of the recovery phase)
z8  = 1.3; % First intermediate waypoint(recovery phase)
z9  = 1.2; % Second intermediate waypoint(recovery phase)
z10 = 1.1; % Third intermediate waypoint(recovery phase)
z11 = 1.0; % Fourth intermediate waypoint(recovery phase)
z12 = 0.9; % Final position (end of recovery phase)

% Velocities along z
z2d  = 1; % Velocity of the first waypoint in the reaching phase
z3d  = 1; % Velocity of the second waypoint in the reaching phase
z4d  = 1; % Velocity of the third waypoint in the reaching phase
z5d  = 1; % Velocity of the fourth waypoint in the reaching phase
z8d  = 1; % Velocity of the first waypoint in the recovery phase
z9d  = 1; % Velocity of the second waypoint in the recovery phase
z10d = 1; % Velocity of the third waypoint in the recovery phase
z11d = 1; % Velocity of the fourth waypoint in the recovery phase


% Accelerations along z
z2dd  = 1; % Acceleration of the first waypoint in the reaching phase
z3dd  = 1; % Acceleration of the second waypoint in the reaching phase
z4dd  = 1; % Acceleration of the third waypoint in the reaching phase
z5dd  = 1; % Acceleration of the fourth waypoint in the reaching phase
z8dd  = 1; % Acceleration of the first waypoint in the recovery phase
z9dd  = 1; % Acceleration of the second waypoint in the recovery phase
z10dd = 1; % Acceleration of the third waypoint in the recovery phase
z11dd = 1; % Acceleration of the fourth waypoint in the recovery phase

% For phi
phi1  = 0; % fixed
phi2  = D2R(17.5);  % First intermediate waypoint (reaching phase)
phi3  = D2R(35);    % Second intermediate waypoint (reaching phase)
phi4  = D2R(52.5);  % third intermediate waypoint (reaching phase)
phi5  = D2R(70);    % Fourth intermediate waypoint (reaching phase)
phi6  = pi/2 - 0.1; % Final position in the reaching phase (start of flip)
phi7  = (3/2)* pi + 0.1; % End of flip (start of recovery phase)
phi8  = D2R(287.5); % First intermediate waypoint (recovery phase)
phi9  = D2R(305);   % Second intermediate waypoint (recovery phase)
phi10 = D2R(322.5); % Third intermediate waypoint (recovery phase)
phi11 = D2R(340);   % Fourth intermediate waypoint (recovery phase)
phi12 = 2 * pi;     % Final position (end of recovery phase)

% Angular velocities
phi2d  = 1; % Angular velocity of the first waypoint in the reaching phase
phi3d  = 1; % Angular velocity of the second waypoint in the reaching phase
phi4d  = 1; % Angular velocity of the third waypoint in the reaching phase
phi5d  = 1; % Angular velocity of the fourth waypoint in the reaching phase
phi8d  = 1; % Angular velocity of the first waypoint in the recovery phase
phi9d  = 1; % Angular velocity of the second waypoint in the recovery phase
phi10d = 1; % Angular velocity of the third waypoint in the recovery phase
phi11d = 1; % Angular velocity of the fourth waypoint in the recovery phase

% Angular accelerations
phi2dd  = 1; % Angular acceleration of the first waypoint in the reaching phase
phi3dd  = 1; % Angular acceleration of the second waypoint in the reaching phase
phi4dd  = 1; % Angular acceleration of the third waypoint in the reaching phase
phi5dd  = 1; % Angular acceleration of the fourth waypoint in the reaching phase
phi8dd  = 1; % Angular acceleration of the first waypoint in the recovery phase
phi9dd  = 1; % Angular acceleration of the second waypoint in the recovery phase
phi10dd = 1; % Angular acceleration of the third waypoint in the recovery phase
phi11dd = 1; % Angular acceleration of the fourth waypoint in the recovery phase

% For the time for each trajectory
t1  = 0.2; % time of the first trajectory (reaching phase)
t2  = 0.2; % time of the second trajectory (reaching phase)
t3  = 0.2; % time of the third trajectory (reaching phase)
t4  = 0.2; % time of the fourth trajectory (reaching phase)
t5  = 0.2; % time of the fifth trajectory (reaching phase)
t6  = 0.2; % time of the flip trajectory
t7  = 0.2; % time of the first trajectory (recovery phase)
t8  = 0.2; % time of the second trajectory (recovery phase)
t9  = 0.2; % time of the third trajectory (recovery phase)
t10 = 0.2; % time of the fourth trajectory (recovery phase)
t11 = 0.2; % time of the fifth trajectory (recovery phase)
%%
x0 = [ z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z2d z3d z4d z5d z8d z9d z10d z11d z2dd z3dd z4dd z5dd z8dd z9dd z10dd z11dd phi2 phi3 phi4 phi5 phi6 phi7 phi8 phi9 phi10 phi11 phi2d phi3d phi4d phi5d phi8d phi9d phi10d phi11d phi2dd phi3dd phi4dd phi5dd phi8dd phi9dd phi10dd phi11dd t1 t2 t3 t4 t5 t6 t7 t8 t9 t10 t11];
%options  = optimset('Display', 'iter', 'Tolx', 1e-14, 'Tolfun',...
%                    1e-14, 'MaxIter', 1e20, 'MaxFunEvals', 1e20);

options  = optimset('Display', 'iter', 'MaxIter', 1e20, 'MaxFunEvals', 1e20);
                
% fmincon optimization
x = fmincon(obj,x0,[],[],[],[],lb,ub,nl_con,options);

% solution of the optimization problem
z1      = x(1);
z2      = x(2);
z3      = x(3);
z4      = x(4);
z5      = x(5);
z6      = x(6);
z7      = x(7);
z8      = x(8);
z9      = x(9);
z10     = x(10);
z11     = x(11);
z12     = x(12);
z2d     = x(13);
z3d     = x(14);
z4d     = x(15);
z5d     = x(16);
z8d     = x(17);
z9d     = x(18);
z10d    = x(19);
z11d    = x(20);
z2dd    = x(21);
z3dd    = x(22);
z4dd    = x(23);
z5dd    = x(24);
z8dd    = x(25);
z9dd    = x(26);
z10dd   = x(27);
z11dd   = x(28);
phi2    = x(29);
phi3    = x(30);
phi4    = x(31);
phi5    = x(32);
phi6    = x(33);
phi7    = x(34);
phi8    = x(35);
phi9    = x(36);
phi10   = x(37);
phi11   = x(38);
phi2d   = x(39);
phi3d   = x(40);
phi4d   = x(41);
phi5d   = x(42);
phi8d   = x(43);
phi9d   = x(44);
phi10d  = x(45);
phi11d  = x(46);
phi2dd  = x(47);
phi3dd  = x(48);
phi4dd  = x(49);
phi5dd  = x(50);
phi8dd  = x(51);
phi9dd  = x(52);
phi10dd = x(53);
phi11dd = x(54);
t1      = round(x(55),2);
t2      = round(x(56),2);
t3      = round(x(57),2);
t4      = round(x(58),2);
t5      = round(x(59),2);
t6      = round(x(60),2);
t7      = round(x(61),2);
t8      = round(x(62),2);
t9      = round(x(63),2);
t10     = round(x(64),2);
t11     = round(x(65),2);

%% Building trajectory
build_trajectory;
%visualize_trajectory;

%% Write trajectory to file

% py = y';
% pz = z';
% roll = phi';
% vy = yd';
% vz = zd';
% rolld = phid';
% 
% ref_X = [py pz roll vy vz rolld];
% ref_U = [u1' u2'];
% %% 
% dlmwrite('saved_data/measX.csv',ref_X);
% dlmwrite('saved_data/simU.csv',ref_U);

