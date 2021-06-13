function traj = trajectory_5(start,goal,t)

    global t_step;

    [A,b] = build_matrix(start,goal,t);
    
    [U,S,V] = svd(A);
    
    A_inv = V * ( S \ U' );
    
    coeff_p = fliplr((A_inv*b)');    
 
    % t = t1:t_step:t1+t2;
    t = 0:t_step:t;
    
    coeff_v = polyder(coeff_p); % coefficients of the velocity polynomial
    coeff_a = polyder(coeff_v); % coefficients of the acceleration polynomial
    % coeff_j = polyder(coeff_a); % coefficients of the jerk polynomial (derivative of the acceleration)
    % coeff_s = polyder(coeff_j); % coefficients of the snap polynomial (derivative of the jerk)
    
    % Resulting trajectory
    traj = [polyval(coeff_p,t);
            polyval(coeff_v,t);
            polyval(coeff_a,t)];
            % polyval(coeff_j,t);
            % polyval(coeff_s,t)];
    
end
