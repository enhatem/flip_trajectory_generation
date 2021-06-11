clear all;
close all;
clc;

%% Constant Parameters

global g t_step z1_min z2_min z3_min z1_max z2_max z3_max m Ixx l flips u1_max u2_max

% Drone parameters
m =  29e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
t_step = 0.01; % 100 Hz
g = 9.81; % m/s^2

% Bounds on the z trajectory of the reaching phase
z1_min = 0.8;
z1_max = 1.5;

% Bounds on the z trajectory of the flipping phase
z2_min = 0.8;
z2_max = 1.5;

% Bounds on the z trajectory of the recovery phase
z3_min = 0.8;
z3_max = 1.5;

% Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 46e-3 * g ) / 2 ); % Maximum thrust 
u2_max = 0.1 * ( 1 / 2 * u1_max * l); % Maximum torque

% Number of flips to be perfomed by the drone
flips  = 1; 


%% Constraints applied on the optimization problem

% lower bound on the initial phi angle at the start of the reaching phase
phi_reaching_start = 0;

% upper bound on the final phi angle at the end of the reaching phase
phi_reaching_end = pi/2;

% lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_recovery_start = (2*(flips-1)*pi+3/2*pi);

% upper bound on the final phi angle at the end of the recovery phase
phi_recovery_end   = 2*flips*pi;

% bounds on t1 (time of the reaching phase trajectory)
t1_min = 0.1;
t1_max = inf;

% bounds on t2 (time of the flipping phase trajectory)
t2_min = 0.1;
t2_max = inf;

% bounds on t3 (time of the recovery phase trajectory)
t3_min = 0.1;
t3_max = inf;

% lower and upper bounds
lb = [ z1_min   z1_min   z2_min  z3_min  phi_reaching_start  phi_recovery_start  t1_min  t2_min  t3_min ];
ub = [ z1_max   z1_max   z2_max  z3_max  phi_reaching_end    phi_recovery_end    t1_max  t2_max  t3_max ];

% nonlinear bounds
nl_con = @NL_bounds_simple;

% objective function
min_time = @objective_function;

%% Optimization problem



% Initial condition
% x0 = [3.1 3.4 1.5 1.1 pi/2-0.2 (3/2+0.2)*flips*pi 0.2 2 0.2]
x0 = [ 1.2    1.4   1.1    0.9    pi/2-0.2    (3/2+0.2)*flips*pi    0.4   0.3  0.4 ];
options  = optimset('Display', 'iter', 'Tolx', 1e-14, 'Tolfun',...
                    1e-14, 'MaxIter', 1e20, 'MaxFunEvals', 1e20);

solution = fmincon(min_time,x0,[],[],[],[],lb,ub,nl_con,options);

z_hover1    = solution(1);
z_start     = solution(2);
z_end       = solution(3);
z_hover2    = solution(4);
phi_start   = solution(5);
phi_end     = solution(6);
% t1          = solution(7);
% t2          = solution(8);
% t3          = solution(9);
t1          = round(solution(7),2);
t2          = round(solution(8),2);
t3          = round(solution(9),2);

%% Visualize results

build_trajectory_simple;

figure, plot(z), title('z(t)')
figure, plot(u1), title('u1(t)')
% build_trajectory;
% visualize_results;
% visualize_trajectory;





%% Write trajectory to file
% 
% py = y';
% pz = z';
% roll = phi';
% vy = yd;
% vz = zd';
% rolld = phid';
% 
% ref_X = [py pz roll vy vz rolld];
% ref_U = [u1' u2'];
% %% 
% dlmwrite('saved_data/measX.csv',ref_X);
% dlmwrite('saved_data/simU.csv',ref_U);
