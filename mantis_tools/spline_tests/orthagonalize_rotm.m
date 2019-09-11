function [ Ro ] = orthagonalize_rotm( R )
%NORMALIZE_ROTM Summary of this function goes here
%   Detailed explanation goes here

    Ro = zeros(size(R));

    for i=1:3
        Ro(1:3,i) = R(1:3,i) / norm(R(1:3,i));
    end
end