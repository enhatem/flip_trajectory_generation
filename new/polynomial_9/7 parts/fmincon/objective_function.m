function J = objective_function(x)
    
    global g t_step m Ixx
    
    z1      = x(1);
    z2      = x(2);
    z3      = x(3);
    z4      = x(4);
    z5      = x(5);
    z6      = x(6);
    z7      = x(7);
    z8      = x(8);
    z2d     = x(9);
    z3d     = x(10);
    z6d     = x(11);
    z7d     = x(12);
    z2dd    = x(13);
    z3dd    = x(14);
    z6dd    = x(15);
    z7dd    = x(16);
    phi2    = x(17);
    phi3    = x(18);
    phi4    = x(19);
    phi5    = x(20);
    phi6    = x(21);
    phi7    = x(22);
    phi2d   = x(23);
    phi3d   = x(24);
    phi6d   = x(25);
    phi7d   = x(26);
    phi2dd  = x(27);
    phi3dd  = x(28);
    phi6dd  = x(29);
    phi7dd  = x(30);
    t1      = x(31);
    t2      = x(32);
    t3      = x(33);
    t4      = x(34);
    t5      = x(35);
    t6      = x(36);
    t7      = x(37);

    %% Trajectory waypoints for the z trajectory

    % For z
    Z1 = [z1 0 0 0 0]; % Initial position (start of the reaching phase)
    Z2 = [z2 z2d z2dd 0 0]; % First waypoint in the reaching phase
    Z3 = [z3 z3d z3dd 0 0]; % Second waypoint in the reaching phase
    Z4 = [z4 ((z5-z4)/t4 + g*t4/2) -g 0 0]; % Final position in the reaching phase (start of the flip phase)
    Z5 = [z5 ((z5-z4)/t4 - g*t4/2) -g 0 0]; % Final position in the flip phase (start of the recovery phase)
    Z6 = [z6 z6d z6dd 0 0]; % First waypoint in the recovery phase
    Z7 = [z7 z7d z7dd 0 0]; % Second waypoiny in the recovery phase
    Z8 = [z8 0 0 0 0]; % Final position in the recovery phase

    % For phi
    PHI1 = [0 0 0 0 0];
    PHI2 = [phi2 phi2d phi2dd 0 0];
    PHI3 = [phi3 phi3d phi3dd 0 0];
    PHI4 = [phi4 (phi5 - phi4)/t4 0 0 0];
    PHI5 = [phi5 (phi5 - phi4)/t4 0 0 0];
    PHI6 = [phi6 phi6d phi6dd 0 0];
    PHI7 = [phi7 phi7d phi7dd 0 0];
    PHI8 = [2*pi 0 0 0 0];

    %% Reaching phase

    % For z
    traj1_z = trajectory(Z1,Z2,t1);
    traj2_z = trajectory(Z2,Z3,t2);
    traj3_z = trajectory(Z3,Z4,t3);

    % For phi
    traj1_phi = trajectory(PHI1,PHI2,t1);
    traj2_phi = trajectory(PHI2,PHI3,t2);
    traj3_phi = trajectory(PHI3,PHI4,t3);
    
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
    %% Recovery phase

    % For z
    traj5_z = trajectory(Z5,Z6,t5);
    traj6_z = trajectory(Z6,Z7,t6);
    traj7_z = trajectory(Z7,Z8,t7);

    % For phi
    traj5_phi = trajectory(PHI5,PHI6,t5);
    traj6_phi = trajectory(PHI6,PHI7,t6);
    traj7_phi = trajectory(PHI7,PHI8,t7);
    
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
    
    %% Building the z and phi trajectories
    % T = 0:t_step:t1+t2+t3+t4+t5+t6+t7;
    z = [traj1_z(1,:) traj2_z(1,:) traj3_z(1,:) traj4_z(1,:) traj5_z(1,:) traj6_z(1,:) traj7_z(1,:)];
    phi = [traj1_phi(1,:) traj2_phi(1,:) traj3_phi(1,:) traj4_phi(1,:) traj5_phi(1,:) traj6_phi(1,:) traj7_phi(1,:)];
    phidd = [traj1_phi(3,:) traj2_phi(3,:) traj3_phi(3,:) traj4_phi(3,:) traj5_phi(3,:) traj6_phi(3,:) traj7_phi(3,:)];

    %% Calculating the thrust u1 along the trajectory

    zdd = [traj1_z(3,:) traj2_z(3,:) traj3_z(3,:) traj4_z(3,:) traj5_z(3,:) traj6_z(3,:) traj7_z(3,:)];

    gravity = g*ones(size(z));

    u1 = m*(zdd+gravity)./cos(phi);
    u2 = Ixx * phidd;
    T = linspace(0,t1+t2+t3+t4+t5+t6+t7,length(z));
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
    
    % An option could be to add the integral of phi^2
    % J = trapz(T,y)^2 + 200*trapz(T,u1)^2 + (t1 +t2 +t3 +t4 +t5 +t6 +t7);
    % J = 100*trapz(T,u1)^2 + t1 + t2 +t3 +t4 +t5 + t6 + t7;
    % J  = trapz(T,u1)^2;
    % J = 0.1*trapz(T,y)^2 + 0.1*trapz(T,z)^2 + 150*trapz(T,u1)^2 + (t1 + t2 +t3 +t4 +t5 + t6 + t7);
    % J = 0.1*trapz(T,y)^2 + 0.1*trapz(T,z)^2 + 500*trapz(T,u1)^2;
    % J = trapz(T,y)^2 + trapz(T,z)^2 + 250*trapz(T,u1)^2;
    J = 100*trapz(T,u1)^2 + trapz(T,u2)^2;
end
