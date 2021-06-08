function y = integrate(x,x0)

    global step;
    
    y = zeros(size(x));
    for i = 1:length(x)
       if i == 1
           y(i) = x0;
       else
           y(i) = y(i-1) + x(i)*step;
       end
    end

end
