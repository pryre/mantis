function [ x ] = vee_down( x_vee )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if size(x_vee) ~= size(zeros(3))
        error('Input matrix must be of size 3x3')
    end
    
%     if (x_vee(1,1) ~= 0) || (x_vee(2,2) ~= 0) || (x_vee(3,3) ~= 0) || ...
%        (x_vee(3,2) ~= -x_vee(2,3)) || ...
%        (x_vee(1,3) ~= -x_vee(3,1)) || ...
%        (x_vee(2,1) ~= -x_vee(1,2))
%        
%         disp(x_vee)
%         error('Input matrix is malformed')
%     end
    
    x = [x_vee(3,2); x_vee(1,3); x_vee(2,1)];
end

