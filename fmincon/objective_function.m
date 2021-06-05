function J = objective_function(x)
    
    global g Step flips m
    
    z_hover1    = x(1);
    z_start     = x(2);
    z_end       = x(3);
    z_hover2    = x(4);
    phi_start   = x(5);
    phi_end     = x(6);
    t1          = x(7);
    t2          = x(8);
    t3          = x(9);



    %% Reaching phase

    z1_start = [z_hover1 0 0 0 0];
    z1_end = [z_start ((z_start-z_end)/t2 + g*t2/2) -g 0 0];
    z1 = trajectory(z1_start,z1_end,t1);

    phi1_start = [0 0 0 0 0];
    phi1_end = [phi_start (phi_end-phi_start)/t2 0 0 0];
    phi1 = trajectory(phi1_start,phi1_end,t1);

    %% Flip phase

    coeff_z2 = [-g/2 ((z_end-z_start)/t2+g*t2/2) z_start];
    coeff_zd2 = polyder(coeff_z2);
    coeff_zdd2 = polyder(coeff_zd2);
    zdd2 = polyval(coeff_zdd2,Step:Step:t2);

    coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
    phi2 = polyval(coeff_phi2,Step:Step:t2);

    %% Recovery phase

    z3_start = [z_end ((z_start-z_end)/t2-g*t2/2) -g 0 0];
    z3_end = [z_hover2 0 0 0 0];
    z3 = trajectory(z3_start,z3_end,t3);

    phi3_start = [phi_end (phi_end-phi_start)/t2 0 0 0];
    phi3_end = [2*flips*pi 0 0 0 0];
    phi3 = trajectory(phi3_start,phi3_end,t3);

    %% Global trajectory

    zdd = [z1(3,:) zdd2 z3(3,:)];

    phi = [phi1(1,:) phi2 phi3(1,:)];

    gravity = g*ones(size(zdd));


    u1 = m*(zdd+gravity)./cos(phi);

    % objective function: J = t1 + t2 + t3
    J = mean(u1.^2); % x(7) + x(8) + x(9);
    % J = x(7) + x(8) + x(9);
end
