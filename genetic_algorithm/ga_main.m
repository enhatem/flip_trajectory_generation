clear; 
close all;
clc;

%% Constant Parameters

global g step z1_min z2_min z3_min z1_max z2_max z3_max y_max m Ixx l flips u1_max u2_max

% Drone parameters
m =  27e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
step = 0.01; % 100 Hz
g = 9.81;    % m/s^2

% Bounds on the z trajectory of the reaching phase
z1_min = 2.5;
z1_max = 3.5;

% Bounds on the z trajectory of the flipping phase
z2_min = 1;
z2_max = 3.5;

% Bounds on the z trajectory of the recovery phase
z3_min = 1;
z3_max = 2;

% Bound on y
y_max = 2;

% Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 57e-3 * g ) / 2 ); % Maximum thrust 
u2_max = 0.1 * ( 1 / 2 * u1_max * l); % Maximum torque

% Number of flips to be perfomed by the drone
flips  = 1;

%% Constraints applied on the optimization problem

% lower bound on the initial phi angle at the start of the reaching phase
phi_reaching_start = 0;

% upper bound on the final phi angle at the end of the reaching phase
phi_reaching_end   = pi/2;

% lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_recovery_start = (2*(flips-1)*pi+3/2*pi);

% upper bound on the final phi angle at the end of the recovery phase
phi_recovery_end   = 2*flips*pi;

% bounds on t1 (time of the reaching phase trajectory)
t1_min = 1e-1;
t1_max = inf;

% bounds on t2 (time of the flipping phase trajectory)
t2_min = 1e-1;
t2_max = inf;

% bounds on t3 (time of the recovery phase trajectory)
t3_min = 1e-1;
t3_max = inf;

% lower and upper bounds
lb = [ z1_min   z1_min   z2_min  z3_min  phi_reaching_start  phi_recovery_start  t1_min  t2_min  t3_min ];
ub = [ z1_max   z1_max   z2_max  z3_max  phi_reaching_end    phi_recovery_end    t1_max  t2_max  t3_max ];

% nonlinear bounds
nonlcon = @NL_bounds;

% objective function
fun = @objective_function;

%% Solving the optimization problem using genetic algorithm
rng(230)
options = optimoptions('ga','ConstraintTolerance',1e-20,'PlotFcn', @gaplotbestf, ...
                       'Display','iter');
% x = ga(fun,9, [], [], [], [], lb, ub, nonlcon, options)
[x,fval,exitflag,output,population,scores] = ga(fun,9,[],[],[],[],lb,ub,nonlcon,options);

z_hover1    = x(1);
z_start     = x(2);
z_end       = x(3);
z_hover2    = x(4);
phi_start   = x(5);
phi_end     = x(6);
t1          = x(7);
t2          = x(8);
t3          = x(9);

%% Building and plotting the trajectory
build_trajectory;
visualize_results;

%% Trajectory plot
figure
plot(y1,z1(1,:),'LineWidth',1.5,'Color','b'),
hold on, plot(y2,z2(1,:),'LineWidth',1.5,'Color','r'),
hold on, plot(y3,z3(1,:),'LineWidth',1.5,'Color','y'),
hold on, plot(0,z_hover1,'*g'), hold on, plot(y1(length(y1)),z_start,'*g'),
hold on, plot(y2(length(y2)),z_end,'*g'),
hold on, plot(y3(length(y3)),z_hover2,'*g'),
for i = 1:10:length(y)
    
   t = phi(i);
   
   q1 = y(i)-l*cos(t);
   q2 = y(i)+l*cos(t);
   w1 = z(i)-l*sin(t);
   w2 = z(i)+l*sin(t);
   
   r1y = q1+r*cos(t+pi/2);
   r1z = w1+r*sin(t+pi/2);
   r2y = q2+r*cos(t+pi/2);
   r2z = w2+r*sin(t+pi/2);

   plot([q1 q2],[w1 w2],'k','LineWidth',1.5), hold on
   plot([q1 r1y],[w1 r1z],'k','LineWidth',1.5), hold on
   plot([q2 r2y],[w2 r2z],'k','LineWidth',1.5), hold on
   
end
title('Planar flipping trajectory'), daspect([1 1 1]), grid;

