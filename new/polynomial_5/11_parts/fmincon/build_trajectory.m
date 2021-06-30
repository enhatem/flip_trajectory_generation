%% Trajectory waypoints for the z trajectory

% For z
Z1 = [z1 0 0]; % Initial position (start of the reaching phase)
Z2 = [z2 z2d z2dd]; % First waypoint in the reaching phase
Z3 = [z3 z3d z3dd]; % Second waypoint in the reaching phase
Z4 = [z4 z4d z4dd]; % Third waypoiny in the reaching phase
Z5 = [z5 z5d z5dd]; % Fourth waypoint in the reaching phase
Z6 = [z6 ((z6-z5)/t6 + g*t6/2) -g]; % Final position in the reaching phase (start of the flip phase)
Z7 = [z7 ((z6-z5)/t6 - g*t6/2) -g]; % Final position in the flip phase (start of the recovery phase)
Z8 = [z8 z8d z8dd]; % First waypoint in the recovery phase
Z9 = [z9 z9d z9dd]; % Second waypoint in the recovery phase
Z10 = [z10 z10d z10dd]; % Third waypoint in the recovery phase
Z11 = [z11 z11d z11dd]; % Fourth waypoint in the recovery phase
Z12 = [z12 0 0]; % Final position in the recovery phase

% For phi
PHI1 = [0 0 0];  % Initial position (start of the reaching phase)

PHI2 = [phi2 phi2d phi2dd]; % First waypoint in the reaching phase
PHI3 = [phi3 phi3d phi3dd]; % Second waypoint in the reaching phase
PHI4 = [phi4 phi4d phi4dd]; % Third waypoint in the reaching phase
PHI5 = [phi5 phi5d phi5dd]; % Fourth waypoint in the reaching phase

PHI6 = [phi6 (phi7 - phi6)/t6 0]; % Final position in the reaching phase (start of the flip phase)
PHI7 = [phi7 (phi7 - phi6)/t6 0]; % Final position in the flip phase (start of the recovery phase)

PHI8  = [phi8 phi8d phi8dd];    % First waypoint in the recovery phase
PHI9  = [phi9 phi9d phi9dd];    % Second waypoint in the recovery phase
PHI10 = [phi10 phi10d phi10dd]; % Third waypoint in the recovery phase
PHI11 = [phi11 phi11d phi11dd]; % Fourth waypoint in the recovery phase

PHI12 = [2*pi 0 0]; % Final position in the recovery phase


%% Reaching phase

% For z
traj1_z = trajectory(Z1,Z2,t1);
traj2_z = trajectory(Z2,Z3,t2);
traj3_z = trajectory(Z3,Z4,t3);
traj4_z = trajectory(Z4,Z5,t4);
traj5_z = trajectory(Z5,Z6,t5);


% For phi
traj1_phi = trajectory(PHI1,PHI2,t1);
traj2_phi = trajectory(PHI2,PHI3,t2);
traj3_phi = trajectory(PHI3,PHI4,t3);
traj4_phi = trajectory(PHI4,PHI5,t4);
traj5_phi = trajectory(PHI5,PHI6,t5);



T1 = 0:t_step:t1;
T2 = t1:t_step:t1+t2;
T3 = t1+t2:t_step:t1+t2+t3;
T4 = t1+t2+t3:t_step:t1+t2+t3+t4;
T5 = t1+t2+t3+t4:t_step:t1+t2+t3+t4+t5;


%% Flip phase

% For z
coeff_z6 = [-g/2 ((z7-z6)/t6+g*t6/2) z6];
coeff_z6d = polyder(coeff_z6);
coeff_z6dd = polyder(coeff_z6d);

traj_z6   = polyval(coeff_z6,0:t_step:t6);
traj_z6d  = polyval(coeff_z6d, 0:t_step:t6); 
traj_z6dd = polyval(coeff_z6dd,0:t_step:t6);

traj6_z =[traj_z6;
          traj_z6d;
          traj_z6dd];
      
% For phi
coeff_phi6 = [(phi7-phi6)/t6 phi6];
coeff_phi6d = polyder(coeff_phi6);
coeff_phi6dd = polyder(coeff_phi6d);
Phi6 = polyval(coeff_phi6,0:t_step:t6);
Phi6d = polyval(coeff_phi6d,0:t_step:t6);
Phi6dd = polyval(coeff_phi6dd,0:t_step:t6);
traj6_phi = [Phi6;
              Phi6d;
              Phi6dd];
      
T6 = t1+t2+t3+t4+t5:t_step:t1+t2+t3+t4+t5+t6;

%% Recovery phase

% For z
traj7_z  = trajectory(Z7,Z8,t7);
traj8_z  = trajectory(Z8,Z9,t8);
traj9_z  = trajectory(Z9,Z10,t9);
traj10_z = trajectory(Z10,Z11,t10);
traj11_z = trajectory(Z11,Z12,t11);

% For phi
traj7_phi = trajectory(PHI7,PHI8,t7);
traj8_phi = trajectory(PHI8,PHI9,t8);
traj9_phi = trajectory(PHI9,PHI10,t9);
traj10_phi = trajectory(PHI10,PHI11,t10);
traj11_phi = trajectory(PHI11,PHI12,t11);

% Generating time vectors for plotting
T7  = t1+t2+t3+t4+t5+t6:t_step:t1+t2+t3+t4+t5+t6+t7;
T8  = t1+t2+t3+t4+t5+t6+t7:t_step:t1+t2+t3+t4+t5+t6+t7+t8;
T9  = t1+t2+t3+t4+t5+t6+t7+t8:t_step:t1+t2+t3+t4+t5+t6+t7+t8+t9;
T10 = t1+t2+t3+t4+t5+t6+t7+t8+t9:t_step:t1+t2+t3+t4+t5+t6+t7+t8+t9+t10;
T11 = t1+t2+t3+t4+t5+t6+t7+t8+t9+t10:t_step:t1+t2+t3+t4+t5+t6+t7+t8+t9+t10+t11;


%% Plotting z and phi by parts

% Trajectory along z
figure, 
plot(T1,traj1_z(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_z(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_z(1,:),'LineWidth',1.5), hold on
plot(T4,traj4_z(1,:),'LineWidth',1.5), hold on
plot(T5,traj5_z(1,:),'LineWidth',1.5), hold on
plot(T6,traj6_z(1,:),'LineWidth',1.5), hold on
plot(T7,traj7_z(1,:),'LineWidth',1.5), hold on
plot(T8,traj8_z(1,:),'LineWidth',1.5), hold on
plot(T9,traj9_z(1,:),'LineWidth',1.5), hold on
plot(T10,traj10_z(1,:),'LineWidth',1.5), hold on
plot(T11,traj11_z(1,:),'LineWidth',1.5), hold on

title('Trajectory of z(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('z[m]')
% 
% % Trajectory along phi 
figure, 
plot(T1,  traj1_phi(1,:),'LineWidth',1.5), hold on
plot(T2,  traj2_phi(1,:),'LineWidth',1.5), hold on
plot(T3,  traj3_phi(1,:),'LineWidth',1.5), hold on
plot(T4,  traj4_phi(1,:),'LineWidth',1.5), hold on
plot(T5,  traj5_phi(1,:),'LineWidth',1.5), hold on
plot(T6,  traj6_phi(1,:),'LineWidth',1.5), hold on
plot(T7,  traj7_phi(1,:),'LineWidth',1.5), hold on
plot(T8,  traj8_phi(1,:),'LineWidth',1.5), hold on
plot(T9,  traj9_phi(1,:),'LineWidth',1.5), hold on
plot(T10,traj10_phi(1,:),'LineWidth',1.5), hold on
plot(T11,traj11_phi(1,:),'LineWidth',1.5), hold off
% 
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
traj8_z = traj8_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj9_z = traj9_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj10_z = traj10_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj11_z = traj11_z(:,2:end); % removes the common point between the 2 consecutive trajectories

% For phi
traj2_phi  = traj2_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_phi  = traj3_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj4_phi  = traj4_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj5_phi  = traj5_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj6_phi  = traj6_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj7_phi  = traj7_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj8_phi  = traj8_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj9_phi  = traj9_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj10_phi = traj10_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj11_phi = traj11_phi(:,2:end); % removes the common point between the 2 consecutive trajectories


%% 
T = 0:t_step:t1+t2+t3+t4+t5+t6+t7+t8+t9+t10+t11;
z = [traj1_z(1,:) traj2_z(1,:) traj3_z(1,:) traj4_z(1,:) traj5_z(1,:) traj6_z(1,:) traj7_z(1,:) traj8_z(1,:) traj9_z(1,:) traj10_z(1,:) traj11_z(1,:)];
phi = [traj1_phi(1,:) traj2_phi(1,:) traj3_phi(1,:) traj4_phi(1,:) traj5_phi(1,:) traj6_phi(1,:) traj7_phi(1,:) traj8_phi(1,:) traj9_phi(1,:)  traj10_phi(1,:) traj11_phi(1,:)];
% figure, plot(T,z, 'LineWidth',1.5)
% title('Trajectory along z(t)')
% xlabel('time[s]')
% ylabel('z[m]')
% 
% figure, plot(T,phi, 'LineWidth',1.5)
% title('Trajectory along \phi(t)')
% xlabel('time[s]')
% ylabel('\phi[rad]')

%% Calculating the thrust u1 along the trajectory

zdd = [traj1_z(3,:) traj2_z(3,:) traj3_z(3,:) traj4_z(3,:) traj5_z(3,:) traj6_z(3,:) traj7_z(3,:) traj8_z(3,:) traj9_z(3,:) traj10_z(3,:) traj11_z(3,:)];
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

figure, plot(T, u1, 'LineWidth', 1.5)
title('Thrust throughout the trajectory')
xlabel('time[s]')
ylabel('u1[N]')