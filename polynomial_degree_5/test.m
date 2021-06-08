clear; close all; clc;

%% 

global t_step g

t_step = 0.01;
g = 9.81;

%% parameters to be optimized

t1 = 0.5;
t2 = 0.7;
t3 = 0.1;
t4 = 0.4;
t5 = 0.2;
t6 = 0.5;
t7 = 0.7;

z1 = 1.4;
z2 = 1.8;
z3 = 1.7; % z_flip_start
z4 = 1.6; % z_flip_end
z5 = 1.5; 
z6 = 1.3;
z7 = 1.1;

phi1 = 0.34; % 20deg
phi2 = 0.4;
phi3 = pi/2 -0.2; % phi_flip_start
phi4 = 3/2 * pi +0.2; % phi_flip_end
phi5 = 5.2; 
phi6 = 5.91;
phi7 = 2*pi;



%% z trajectory 

zd_des = 2.5 / 3 ;

zdd1_des = (zd_des - 0) / t1;
zdd2_des = (2*zd_des - zd_des) / t2;


Z0 = [1 0 0];
Z1 = [z1 zd_des zdd1_des];
Z2 = [z2 2*zd_des zdd2_des];
Z3 = [z3 ((z4-z3)/t3 - (g*t3)/2) -g ];

traj1 = trajectory_5(Z0,Z1,t1);
T1 = linspace(0,0+t1,length(traj1(1,:)));
figure, plot(T1,traj1(1,:)), hold on

traj2 = trajectory_5(Z1,Z2,t2);
T2 = linspace(0+t1,0+t1+t2,length(traj2(1,:)));
plot(T2,traj2(1,:)), hold on

traj3 = trajectory_5(Z2,Z3,t3);
T3 = linspace(0+t1+t2,0+t1+t2+t3,length(traj3(1,:)));
plot(T3,traj3(1,:)), hold on


% flip
coeff_Z4 = [-g/2 ((z4-z3)/t4+g*t4/2) z3];
coeff_Zd4 = polyder(coeff_Z4);
coeff_Zdd4 = polyder(coeff_Zd4);
traj4 = [ polyval(coeff_Z4,0:t_step:t4);
          polyval(coeff_Zd4,0:t_step:t4);
          polyval(coeff_Zdd4,0:t_step:t4) ];
      

T4 = linspace(0+t1+t2+t3, 0+t1+t2+t3+t4,length(traj4(1,:)));
plot(T4,traj4(1,:)), hold on

% recovery
Z4 = [z4 ((z3-z4)/t4-g*t4/2) -g];
Z5 = [z5 ((z3-z4)/t4-g*t4/2)/2  -g/2];
Z6 = [z6 ((z3-z4)/t4-g*t4/2)/4  -g/4];
Z7 = [z7 0 0];

traj5 = trajectory_5(Z4,Z5,t5);
T5 = linspace(0+t1+t2+t3+t4, 0+t1+t2+t3+t4+t5,length(traj5(1,:)));
plot(T5,traj5(1,:)), hold on

traj6 = trajectory_5(Z5,Z6,t6);
T6 = linspace(0+t1+t2+t3+t4+t5, 0+t1+t2+t3+t4+t5+t6,length(traj6(1,:)));
plot(T6,traj6(1,:)), hold on

traj7 = trajectory_5(Z6,Z7,t7);
T7 = linspace(0+t1+t2+t3+t4+t5+t6, 0+t1+t2+t3+t4+t5+t6+t7,length(traj7(1,:)));
plot(T7,traj7(1,:)), hold on