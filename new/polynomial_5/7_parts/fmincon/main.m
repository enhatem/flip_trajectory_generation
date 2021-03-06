clear; close all; clc;

%% Drone parameters and costants

global t_step g;

% Drone parameters
m =  29e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
t_step =0.01;
g = 9.81; % m/s^2

%% Trajectory parameters (to be optimized)

% For z
z1 = 1.0; % initial position
z2 = 1.1; % z at the end of the first trajectory (reaching phase)
z3 = 1.3; % z at the end of the second trajectory (reaching phase)
z4 = 1.5; % z at the end of the third trajectory (end of reaching phase, beginning of flip phase)
z5 = 1.3; % z at the end of the fourth trajectory (end of flip phase)
z6 = 1.1; % z at the end of the fifth trajectory (recovery phase)
z7 = 1.0; % z at the end of the sixth trajectory (recovery phase)
z8 = 0.9; % z at the end of the seventh trajectory (recovery phase)

% Velocities along z
z2d = 1 ; % Velocity of the first waypoint in the reaching phase
z3d = 1 ; % Velocity of the second waypoint in the reaching phase
z6d = 1 ; % Velocity of the first waypoint in the recovery phase
z7d = 1 ; % Velocity of the second waypoint in the recovery phase

% Accelerations along z
z2dd = 1; % Acceleration of the first waypoint in the reaching phase
z3dd = 1; % Acceleration of the second waypoint in the reaching phase
z6dd = 1; % Acceleration of the first waypoint in the recovery phase
z7dd = 1; % Acceleration of the second waypoint in the recovery phase

% For phi
phi1 = 0; % fixed
phi2 = D2R(28.33); % phi at the end of the first trajectory (reaching phase)
phi3 = D2R(56.67); % phi at the end of the second trajectory (reaching phase)
phi4 = pi/2 - 0.1; % phi at the end of the third trajectory (end of reaching phase, beginning of flip phase)
phi5 = (3/2)* pi + 0.1; % phi at the end of the fourth trajectory (end of flip phase)
phi6 = D2R(303.33); % phi at the end of the fifth trajectory (recovery phase)
phi7 = D2R(331.66); % phi at the end of the sixth trajectory (recovery phase)
phi8 = D2R(2*pi); % phi at the end of the seventh trajectory (recovery phase)

% Angular velocities
phi2d = 1; % Angular velocity of the first waypoint in the reaching phase
phi3d = 1; % Angular velocity of the second waypoint in the reaching phase
phi6d = 1; % Angular velocity of the first waypoint in the recovery phase
phi7d = 1; % Angular velocity of the second waypoint in the recovery phase

% Angular accelerations
phi2dd = 1; % Angular acceleration of the first waypoint in the reaching phase
phi3dd = 1; % Angular acceleration of the second waypoint in the reaching phase
phi6dd = 1; % Angular acceleration of the first waypoint in the recovery phase
phi7dd = 1; % Angular acceleration of the second waypoint in the recovery phase

% For the time for each trajectory
t1 = 0.3; % time of the first trajectory (reaching phase)
t2 = 0.2; % time of the second trajectory (reaching phase)
t3 = 0.5; % time of the third trajectory (reaching phase)
t4 = 0.2; % time of the fourth trajectory (flip phase)
t5 = 0.3; % time of the fifth trajectory (recovery phase)
t6 = 0.4; % time of the sixth trajectory (recovery phase)
t7 = 0.3; % time of the seventh trajectory (recovery phase)

%% Trajectory waypoints for the z trajectory

% For z
Z1 = [z1 0 0]; % Initial position (start of the reaching phase)
Z2 = [z2 z2d z2dd]; % First waypoint in the reaching phase
Z3 = [z3 z3d z3dd]; % Second waypoint in the reaching phase
Z4 = [z4 ((z5-z4)/t4 + g*t4/2) -g]; % Final position in the reaching phase (start of the flip phase)
Z5 = [z5 ((z5-z4)/t4 - g*t4/2) -g]; % Final position in the flip phase (start of the recovery phase)
Z6 = [z6 z6d z6dd]; % First waypoint in the recovery phase
Z7 = [z7 z7d z7dd]; % Second waypoiny in the recovery phase
Z8 = [z8 0 0]; % Final position in the recovery phase

% For phi
PHI1 = [0 0 0];
PHI2 = [phi2 phi2d phi2dd];
PHI3 = [phi3 phi3d phi3dd];
PHI4 = [phi4 (phi5 - phi4)/t4 0];
PHI5 = [phi5 (phi5 - phi4)/t4 0];
PHI6 = [phi6 phi6d phi6dd];
PHI7 = [phi7 phi7d phi7dd];
PHI8 = [2*pi 0 0];


%% Reaching phase

% For z
traj1_z = trajectory(Z1,Z2,t1);
traj2_z = trajectory(Z2,Z3,t2);
traj3_z = trajectory(Z3,Z4,t3);

% For phi
traj1_phi = trajectory(PHI1,PHI2,t1);
traj2_phi = trajectory(PHI2,PHI3,t2);
traj3_phi = trajectory(PHI3,PHI4,t3);


T1 = 0:t_step:t1;
T2 = t1:t_step:t1+t2;
T3 = t1+t2:t_step:t1+t2+t3;

%% Flip phase

% For z
coeff_z4 = [-g/2 ((z5-z4)/t4+g*t4/2) z4];
coeff_z4d = polyder(coeff_z4);
coeff_z4dd = polyder(coeff_z4d);

traj_z4   = polyval(coeff_z4,0:t_step:t4);
traj_z4d  = polyval(coeff_z4d, 0:t_step:t4); 
traj_z4dd = polyval(coeff_z4dd,0:t_step:t4);

traj4_z =[traj_z4;
          traj_z4d;
          traj_z4dd];
      
% For phi
coeff_phi4 = [(phi5-phi4)/t4 phi4];
coeff_phi4d = polyder(coeff_phi4);
coeff_phi4dd = polyder(coeff_phi4d);
Phi4 = polyval(coeff_phi4,0:t_step:t4);
Phi4d = polyval(coeff_phi4d,0:t_step:t4);
Phi4dd = polyval(coeff_phi4dd,0:t_step:t4);
traj4_phi = [Phi4;
             Phi4d;
             Phi4dd];
      
T4 = t1+t2+t3:t_step:t1+t2+t3+t4;
%% Recovery phase

% For z
traj5_z = trajectory(Z5,Z6,t5);
traj6_z = trajectory(Z6,Z7,t6);
traj7_z = trajectory(Z7,Z8,t7);

% For phi
traj5_phi = trajectory(PHI5,PHI6,t5);
traj6_phi = trajectory(PHI6,PHI7,t6);
traj7_phi = trajectory(PHI7,PHI8,t7);

T5 = t1+t2+t3+t4:t_step:t1+t2+t3+t4+t5;
T6 = t1+t2+t3+t4+t5:t_step:t1+t2+t3+t4+t5+t6;
T7 = t1+t2+t3+t4+t5+t6:t_step:t1+t2+t3+t4+t5+t6+t7;

%% Plotting z and phi by parts

% Trajectory along z
figure, 
plot(T1,traj1_z(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_z(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_z(1,:),'LineWidth',1.5), hold on
plot(T4,traj4_z(1,:),'LineWidth',1.5), hold on
plot(T5,traj5_z(1,:),'LineWidth',1.5), hold on
plot(T6,traj6_z(1,:),'LineWidth',1.5), hold on
plot(T7,traj7_z(1,:),'LineWidth',1.5), hold off

title('Trajectory of z(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('z[m]')

% Trajectory along phi 
figure, 
plot(T1,traj1_phi(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_phi(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_phi(1,:),'LineWidth',1.5), hold on
plot(T4,traj4_phi(1,:),'LineWidth',1.5), hold on
plot(T5,traj5_phi(1,:),'LineWidth',1.5), hold on
plot(T6,traj6_phi(1,:),'LineWidth',1.5), hold on
plot(T7,traj7_phi(1,:),'LineWidth',1.5), hold off


title('Trajectory of \phi(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('\phi[rad]')

%% removing the identical points

% For z
traj2_z = traj2_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_z = traj3_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj4_z = traj4_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj5_z = traj5_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj6_z = traj6_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj7_z = traj7_z(:,2:end); % removes the common point between the 2 consecutive trajectories

% For phi
traj2_phi = traj2_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_phi = traj3_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj4_phi = traj4_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj5_phi = traj5_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj6_phi = traj6_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj7_phi = traj7_phi(:,2:end); % removes the common point between the 2 consecutive trajectories

%% 
T = 0:t_step:t1+t2+t3+t4+t5+t6+t7;
z = [traj1_z(1,:) traj2_z(1,:) traj3_z(1,:) traj4_z(1,:) traj5_z(1,:) traj6_z(1,:) traj7_z(1,:)];
phi = [traj1_phi(1,:) traj2_phi(1,:) traj3_phi(1,:) traj4_phi(1,:) traj5_phi(1,:) traj6_phi(1,:) traj7_phi(1,:)];

figure, plot(T,z, 'LineWidth',1.5)
title('Trajectory along z(t)')
xlabel('time[s]')
ylabel('z[m]')

figure, plot(T,phi, 'LineWidth',1.5)
title('Trajectory along \phi(t)')
xlabel('time[s]')
ylabel('\phi[rad]')

%% Calculating the thrust u1 along the trajectory

zdd = [traj1_z(3,:) traj2_z(3,:) traj3_z(3,:) traj4_z(3,:) traj5_z(3,:) traj6_z(3,:) traj7_z(3,:)];

gravity = g*ones(size(z));

u1 = m*(zdd+gravity)./cos(phi);

%% Calculating the trajectory along y

% Calculation of ydd
ydd = -tan(phi).*(zdd + gravity);

% Initial conditions for yd and y
yd0 = 0; % initial condition for yd
y0  = 0; % initial condition for y

% Calculation of yd
yd = yd0 + cumtrapz(T,ydd);

% Calculation of y
y = y0 + cumtrapz(T,yd);

plot(T,y,'LineWidth',1.5)
title('Trajectory of y(t)')
xlabel('time[s]')
ylabel('y[m]')


figure, plot(y,z,'LineWidth',1.5)
title('Planar trajectory along the y-z axes')
xlabel('y[m]')
ylabel('z[m]')