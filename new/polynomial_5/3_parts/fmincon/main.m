clear; close all; clc;

%% Drone parameters and costants

global t_step;

% Drone parameters
m =  29e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
t_step =0.01;
g = 9.81; % m/s^2

%% Trajectory parameters (to be optimized)

% For z
z1 = 1;
z2 = 1.5;
z3 = 1.3;
z4 = 1;

% For phi
phi_start = pi/2 - 0.1;
phi_end = (3/2)* pi + 0.1;

% For the time for each trajectory
t1 = 0.3;
t2 = 0.2;
t3 = 0.5;

%% Trajectory waypoints for the z trajectory

% For z
Z1 = [z1 0 0]; % initial position (start of the reaching phase)
Z2 = [z2 ((z3-z2)/t2 + g*t2/2) -g];
Z3 = [z3 ((z3-z2)/t2 - g*t2/2) -g];
Z4 = [z4 0 0];

% For phi
PHI1 = [0 0 0];
PHI2 = [phi_start (phi_end - phi_start)/t2 0];
PHI3 = [phi_end (phi_end - phi_start)/t2 0];
PHI4 = [2*pi 0 0];


%% Reaching phase

% For z
traj1_z = trajectory(Z1,Z2,t1);

% For phi
traj1_phi = trajectory(PHI1,PHI2,t1);
T1 = 0:t_step:t1;

%% Flip phase

% For z
coeff_z2 = [-g/2 ((z3-z2)/t2+g*t2/2) z2];
coeff_zd2 = polyder(coeff_z2);
coeff_zdd2 = polyder(coeff_zd2);

traj_z2   = polyval(coeff_z2,0:t_step:t2);
traj_zd2  = polyval(coeff_zd2, 0:t_step:t2); 
traj_zdd2 = polyval(coeff_zdd2,0:t_step:t2);

traj2_z =[traj_z2;
          traj_zd2;
          traj_zdd2];
      
% For phi
coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
coeff_phid2 = polyder(coeff_phi2);
coeff_phidd2 = polyder(coeff_phid2);
phi2 = polyval(coeff_phi2,0:t_step:t2);
phid2 = polyval(coeff_phid2,0:t_step:t2);
phidd2 = polyval(coeff_phidd2,0:t_step:t2);
traj2_phi = [phi2;
             phid2;
             phidd2];
      
      
T2 = t1:t_step:t1+t2;

%%
% Recovery phase

% For z
traj3_z = trajectory(Z3,Z4,t3);

% For phi
traj3_phi = trajectory(PHI3,PHI4,t3);

T3 = t1+t2:t_step:t1+t2+t3;


%% Plotting z and phi by parts

% Trajectory along z
figure, plot(T1,traj1_z(1,:),'LineWidth',1.5), hold on
plot(T2,traj_z2,'LineWidth',1.5), hold on
plot(T3,traj3_z(1,:),'LineWidth',1.5), hold off

title('Trajectory of z(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('z[m]')

% Trajectory along phi 
figure, plot(T1,traj1_phi(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_phi(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_phi(1,:),'LineWidth',1.5), hold off

title('Trajectory of z(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('phi[rad]')

%% removing the identical points

traj2_z = traj2_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_z = traj3_z(:,2:end); % removes the common point between the 2 consecutive trajectories

traj2_phi = traj2_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_phi = traj3_phi(:,2:end); % removes the common point between the 2 consecutive trajectories

z = [traj1_z(1,:) traj2_z(1,:) traj3_z(1,:)];
phi = [traj1_phi(1,:) traj2_phi(1,:) traj3_phi(1,:)];

T = 0:t_step:t1+t2+t3;

% figure, plot(T,z)

% figure, plot(T,phi)

%% Calculating the thrust u1 along the trajectory

zdd = [traj1_z(3,:) traj2_z(3,:) traj3_z(3,:)];

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