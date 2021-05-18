function [c,ceq] = NL_bounds(x)
%NL_BOUNDS extracts the nonlinear bounds that will be used in the
%optimization problem.

    global g step z1_min z2_min z3_min z1_max z2_max z3_max m Ixx flips u1_max u2_max

    ceq = []; % no equality constraints
    
    z_hover1 = x(1); % initial height of the reaching phase
    z_start  = x(2); % initial height of the flipping phase
    z_end    = x(3); % final   height of the flipping phase
    z_hover2 = x(4); % final   height of the recovery phase

    phi_start = x(5); % initial roll angle of the flipping phase
    phi_end = x(6);   % final   roll angle of the flipping phase
    
    t1 = x(7); % time of the reaching phase
    t2 = x(8); % time of the flipping phase
    t3 = x(9); % time of the recovery phase

    % Reaching phase (z)
    z1_start = [z_hover1 0 0 0 0];
    z1_end   = [z_start ((z_start-z_end)/t2 + g*t2/2) -g 0 0];
    z1       = trajectory(z1_start,z1_end,t1);

    % Reaching phase (phi)
    phi1_start = [0 0 0 0 0];
    phi1_end = [phi_start (phi_end-phi_start)/t2 0 0 0];
    phi1 = trajectory(phi1_start,phi1_end,t1);
    
    
    % Flipping phase (z)
    coeff_z2 = [-g/2 ((z_end-z_start)/t2+g*t2/2) z_start];
    coeff_zd2 = polyder(coeff_z2);
    coeff_zdd2 = polyder(coeff_zd2);
    z2 = polyval(coeff_z2,step:step:t2);
    zdd2 = polyval(coeff_zdd2,step:step:t2);

    % Flipping phase (phi)
    coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
    coeff_phid2 = polyder(coeff_phi2);
    coeff_phidd2 = polyder(coeff_phid2);
    phi2 = polyval(coeff_phi2,step:step:t2);
    phid2 = polyval(coeff_phid2,step:step:t2);
    phidd2 = polyval(coeff_phidd2,step:step:t2);
    
    % Recovery phase
    z3_start = [z_end ((z_start-z_end)/t2-g*t2/2) -g 0 0];
    z3_end = [z_hover2 0 0 0 0];
    z3 = trajectory(z3_start,z3_end,t3);

    phi3_start = [phi_end (phi_end-phi_start)/t2 0 0 0];
    phi3_end = [2*flips*pi 0 0 0 0];
    phi3 = trajectory(phi3_start,phi3_end,t3);
    
    
    % Full trajectory
    z = [z1(1,:) z2 z3(1,:)];
    zdd = [z1(3,:) zdd2 z3(3,:)];

    phi = [phi1(1,:) phi2 phi3(1,:)];
    phid = [phi1(2,:) phid2 phi3(2,:)];
    phidd = [phi1(3,:) phidd2 phi3(3,:)];

    gravity = g*ones(size(z));

    % Input

    u1 = m*(zdd+gravity)./cos(phi);
    u2 = Ixx*phidd;
    
    % Bounds 
    
    c = [   max(z1(1,:))-z1_max;    % upper bound on the reaching phase
           -min(z1(1,:))+z1_min;    % lower bound on the reaching phase
            max(z2(1,:))-z2_max;    % upper bound on the flipping phase
           -min(z2(1,:))+z2_min;    % lower bound on the flipping phase
            max(z3(1,:))-z3_max;    % upper bound on the recovery phase
           -min(z3(1,:))+z3_min;    % lower bound on the recovery phase
           -min(u1);                % lower bound on u1 (u1>=0)
            max(u1)-u1_max;         % upper bound on u1 (u1<=u1 max)
           -u2_max-min(u2);         % lower bound on u2 (u2>=-u2_max)
            max(u2)-u2_max;         % upper bound on u2 (u2<=u2_max)
           -4*pi-min(phid);         % lower bound on thetad (thetad >= thetad_min) (angular velocity for aggressive maneuvers is around 720 deg/s)
            max(phid)-4*pi];        % upper bound on thetad (thetad <= thetad_max) (angular velocity for aggressive maneuvers is around 720 deg/s)
    
    
end

