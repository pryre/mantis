function [ Ick ] = inertial_offset( I, m, ck )
%INERTIAL_OFFSET Summary of this function goes here
%   Inputs:
%       I: Centered inertial matrix
%       m: Mass of object
%       ck: Point of reference
%   Outputs:
%       Ick: Translated inertial matrix

    if size(ck) ~= size(zeros(3,1))
        error('Input ck vector must be of size 3x1')
    end
    
    if size(I) ~= size(zeros(3))
        error('Input I matrix must be of size 3x3')
    end
    
    Ick = I - m*((ck'*ck*eye(3) - ck*ck'));
end

