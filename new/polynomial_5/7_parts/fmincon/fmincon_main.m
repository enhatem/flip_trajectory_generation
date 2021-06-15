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
% bounds on t4 (time of the flip phase trajectory)
t4_min = 0.1;
t4_max = inf;

% bounds on t5 (time of the first trajectory in the recovery phase)
t5_min = 0.1;
t5_max = inf;

% bounds on t6 (time of the second trajectory in the recovery phase)
t6_min = 0.1;
t6_max = inf;

% bounds on t7 (time of the third trajectory in the recovery phase)
t7_min = 0.1;
t7_max = inf;

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
% lb = [ z1_min   z1_min   z2_min  z3_min  phi_start_min  phi_end_min  t1_min  t2_min  t3_min ];
% ub = [ z1_max   z1_max   z2_max  z3_max  phi_start_max  phi_end_max  t1_max  t2_max  t3_max ];

lb = [z1_min z1_min z1_min z1_min z2_min z3_min z3_min z3_min zd_min zd_min zd_min zd_min zdd_min zdd_min zdd_min zdd_min phi_start_min phi_start_min phi_start_min phi_end_min phi_end_min phi_end_min phid_min phid_min phid_min phid_min phidd_min phidd_min phidd_min phidd_min t1_min t2_min t3_min t4_min t5_min t6_min t7_min];
ub = [z1_max z1_max z1_max z1_max z2_max z3_max z3_max z3_max zd_max zd_max zd_max zd_max zdd_max zdd_max zdd_max zdd_max phi_start_max phi_start_max phi_start_max phi_end_max phi_end_max phi_end_max phid_max phid_max phid_max phid_max phidd_max phidd_max phidd_max phidd_max t1_max t2_max t3_max t4_max t5_max t6_max t7_max];

%%

% nonlinear bounds
nl_con = @nonlinear_bounds;

% objective function
obj = @objective_function;

%% Optimization problem

% Initial condition
x0 = [ 1.1    1.49    1.1    0.8    pi/2-0.2    (3/2+0.2)*pi    0.4   0.3  0.4 ];
options  = optimset('Display', 'iter', 'Tolx', 1e-14, 'Tolfun',...
                    1e-14, 'MaxIter', 1e20, 'MaxFunEvals', 1e20);

solution = fmincon(obj,x0,[],[],[],[],lb,ub,nl_con,options);

z1 = solution(1);
z2 = solution(2);
z3 = solution(3);
z4 = solution(4);
phi_start = solution(5);
phi_end   = solution(6);
t1 = round(solution(7),2);
t2 = round(solution(8),2);
t3 = round(solution(9),2);

%% Building trajectory

build_trajectory;
plot_data;