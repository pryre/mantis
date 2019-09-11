function [ I ] = inertial_gen( Ix, Iy, Iz, m )
%INERTIAL_GEN Summary of this function goes here
%   Detailed explanation goes here

    IJ = [Ix, 0, 0; ... %Rotational Inertial Tensor
           0, Iy, 0; ...
           0, 0, Iz];
       
    I = [      IJ, zeros(3); ... %Pose Inertial Tensor
         zeros(3), m*eye(3)];
     
end

