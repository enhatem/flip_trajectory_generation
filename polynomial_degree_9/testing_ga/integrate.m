function y = integrate(x,x0)

    global t_step;
    
    y = zeros(size(x));
    for i = 1:length(x)
       if i == 1
           y(i) = x0;
       else
           y(i) = y(i-1) + x(i)*t_step;
       end
    end

end
