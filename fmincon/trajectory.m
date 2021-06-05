function traj = trajectory(start,goal,time)
 
    global Step;

    [A,b] = build_matrix(start,goal,time);
    coeff_p = fliplr((A\b)');    
 
    t = Step:Step:time;
    % t = 0:step:time;
    
    coeff_v = polyder(coeff_p); % coefficients of the velocity polynomial
    coeff_a = polyder(coeff_v); % coefficients of the acceleration polynomial
    coeff_j = polyder(coeff_a); % coefficients of the jerk polynomial (derivative of the acceleration)
    coeff_s = polyder(coeff_j); % coefficients of the snap polynomial (derivative of the jerk)
    
    % Resulting trajectory
    traj = [polyval(coeff_p,t);
            polyval(coeff_v,t);
            polyval(coeff_a,t);
            polyval(coeff_j,t);
            polyval(coeff_s,t)];
    
end
