function [ x_vee ] = vee_up( x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if size(x) ~= size([0;0;0])
        error('Input vector must be of size 3x1')
    end
    
    x_vee = [0, -x(3), x(2); ...
             x(3), 0, -x(1); ...
             -x(2), x(1), 0];
end

