function [ C ] = fdcc(kernel,order)
%FDCC Finite Difference Coefficients Calculator
% Finite Difference Coefficients
% http://web.media.mit.edu/~crtaylor/calculator.html
% https://en.wikipedia.org/wiki/Finite_difference#Higher-order_differences
%
% 3-sample Kernel
% Kernel: [-1,0,1]
%     (-1*f(x-1h) + 0*f(x+0h)+ 1*f(x+1h))/(2(h^1))
%
% 5-sample Kernel
% Kernel: [-1,0,1,2]
%     (-2*f(x-1h) - 3*f(x+0h) + 6*f(x+1h) - 1*f(x+2h))/(6(h^1))
% Kernel: [-2,-1,0,1,2]
%     (1*f(x-2h) - 8*f(x-1h) + 0*f(x+0h) + 8*f(x+1h) - 1*f(x+2h))/(12(h^1))
% Kernel: [-2,-1,0,1]
%     (1*f(x-2h) - 6*f(x-1h) + 3*f(x+0h) + 2*f(x+1h))/(6(h^1))
%

    n = length(kernel);
    %p = [-2, -1, 0, 1, 2]; % Equal window kernel
    %order = 2; % Second order derivative

    if order >= n
        error('Order must be less than the number of points in the kernel');
    end

    M = zeros(n,n);
    M(1,:) = ones(1,n);
    for i = 2:n
        M(i,:) = M(i-1,:).*kernel;
    end

    d = zeros(n,1);
    d(order+1) = factorial(order);

    C = inv(M)*d;

%     divisor = 1/min(abs(C(C~=0)));
%     coeffs = divisor*C;
%     disp(['Calculated coefficients: ', num2str(coeffs')]);
%     disp(['Calculated divisor: ', num2str(divisor)]);

end

