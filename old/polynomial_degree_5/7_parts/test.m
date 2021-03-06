clear; close all; clc;

%% 

global t_step g

t_step = 0.01;
g = 9.81;

%% parameters to be optimized

t1 = 0.5; % time of the first trajectory of the reaching phase
t2 = 0.5; % time of the second trajectory of the reaching phase
t3 = 0.5; % time of the third trajectory of the reaching phase
t4 = 0.2; % time of the flip trajectory
t5 = 0.2; % time of the first trajectory of the recovery phase
t6 = 0.2; % time of the second trajectory of the recovery phase
t7 = 0.2; % time of the third trajectory of the recovery phase

z0 = 1.0; %initil position
z1 = 1.2; % first  intermediate point in the reaching phase 
z2 = 1.4; % second intermediate point in the reaching phase
z3 = 1.6; % start of the flip phase
z4 = 1.4; % end   of the flip phase
z5 = 1.3; % first  intermediate point in the recovery phase 
z6 = 1.1; % second intermediate point in the recovery phase 
z7 = 1.0; % final position

phi1 = 0.45;          % 26deg
phi2 = 0.90;          % 54deg
phi3 = pi/2 -0.2;     % 80deg (start of the flipping phas)
phi4 = 3/2 * pi +0.2; % 280deg (start of the recovery phase)
phi5 = 5.2;           % 306deg
phi6 = 5.91;          % 332deg
phi7 = 2*pi;          % 360deg



%% z trajectory 

zd_des = 2.5 / 3 ;

zdd1_des = (((z4-z3)/t4)+((g*t4)/2)/3 - 0) / t1;
zdd2_des = (2*((z4-z3)/t4)+((g*t4)/2)/3 - ((z4-z3)/t4)+((g*t4)/2)/3) / t2;


Z0 = [1 0 0];
Z1 = [z1 ((z4-z3)/t4)+((g*t4)/2)/3 zdd1_des];
Z2 = [z2 2*((z4-z3)/t4)+((g*t4)/2)/3 zdd2_des];
Z3 = [z3 ((z4-z3)/t4)+((g*t4)/2) -g ];

traj1 = trajectory_5(Z0,Z1,t1);
T1 = linspace(0,0+t1,length(traj1(1,:)));
figure, plot(T1,traj1(1,:), 'LineWidth',1.5), hold on
%%
traj2 = trajectory_5(Z1,Z2,t2);
T2 = linspace(0+t1,0+t1+t2,length(traj2(1,:)));
plot(T2,traj2(1,:), 'LineWidth',1.5), hold on
%%
traj3 = trajectory_5(Z2,Z3,t3);
T3 = linspace(0+t1+t2,0+t1+t2+t3,length(traj3(1,:)));
plot(T3,traj3(1,:), 'LineWidth',1.5), hold on


% flip
coeff_Z4 = [-g/2 ((z4-z3)/t4+g*t4/2) z3];
coeff_Zd4 = polyder(coeff_Z4);
coeff_Zdd4 = polyder(coeff_Zd4);
traj4 = [ polyval(coeff_Z4,0:t_step:t4);
          polyval(coeff_Zd4,0:t_step:t4);
          polyval(coeff_Zdd4,0:t_step:t4) ];
      

T4 = linspace(0+t1+t2+t3, 0+t1+t2+t3+t4,length(traj4(1,:)));
plot(T4,traj4(1,:), 'LineWidth',1.5), hold on

% recovery
zdd5_des = (2*((z4-z3)/t4-g*t4/2)/3 - ((z4-z3)/t4-g*t4/2) ) / t5;
zdd6_des = ( ((z4-z3)/t4-g*t4/2)/3 - 2*((z4-z3)/t4-g*t4/2)/3) / t6;

Z4 = [z4 ((z4-z3)/t4-g*t4/2) -g];
Z5 = [z5 2*((z4-z3)/t4-g*t4/2)/3  zdd5_des];
Z6 = [z6 ((z4-z3)/t4-g*t4/2)/3  zdd6_des];
Z7 = [z7 0 0];
% Z5 = [z5 ((z4-z3)/t4-g*t5/5)/2  -g/2];
% Z6 = [z6 ((z4-z3)/t4-g*t6/6)/4  -g/4];
% Z7 = [z7 0 0];

traj5 = trajectory_5(Z4,Z5,t5);
T5 = linspace(0+t1+t2+t3+t4, 0+t1+t2+t3+t4+t5,length(traj5(1,:)));
plot(T5,traj5(1,:), 'LineWidth',1.5), hold on

traj6 = trajectory_5(Z5,Z6,t6);
T6 = linspace(0+t1+t2+t3+t4+t5, 0+t1+t2+t3+t4+t5+t6,length(traj6(1,:)));
plot(T6,traj6(1,:), 'LineWidth',1.5), hold on

traj7 = trajectory_5(Z6,Z7,t7);
T7 = linspace(0+t1+t2+t3+t4+t5+t6, 0+t1+t2+t3+t4+t5+t6+t7,length(traj7(1,:)));
plot(T7,traj7(1,:), 'LineWidth',1.5), hold off
title('z(t)')
xlabel('time[s]')
ylabel('z[m]')

traj_tot_p = [traj1(1,:) traj2(1,:) traj3(1,:) traj4(1,:) traj5(1,:) traj6(1,:) traj7(1,:)];
t = linspace(0,t1+t2+t3+t4+t5+t6+t7, length(traj_tot_p(1,:)));

% figure, plot(t,traj_tot_p), title('z(t) trajectory'), xlabel('time[s]'), ylabel('z[m]')



%% velocity
traj_tot_v = [traj1(2,:) traj2(2,:) traj3(2,:) traj4(2,:) traj5(2,:) traj6(2,:) traj7(2,:)];

% figure, plot(t,traj_tot_v), title('vz(t) trajectory')

figure, plot(T1,traj1(2,:), 'LineWidth',1.5), hold on
plot(T2,traj2(2,:), 'LineWidth',1.5), hold on
plot(T3,traj3(2,:), 'LineWidth',1.5), hold on
plot(T4,traj4(2,:), 'LineWidth',1.5), hold on
plot(T5,traj5(2,:), 'LineWidth',1.5), hold on
plot(T6,traj6(2,:), 'LineWidth',1.5), hold on
plot(T7,traj7(2,:), 'LineWidth',1.5), hold off
title('vz(t)')
xlabel('time[s]')
ylabel('vz[m]')
