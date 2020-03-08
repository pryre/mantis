% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ I ] = inertial_gen( m, Ix, Iy, Iz )
%INERTIAL_GEN Summary of this function goes here
%   Detailed explanation goes here

    IJ = [Ix, 0, 0; ... %Rotational Inertial Tensor
           0, Iy, 0; ...
           0, 0, Iz];
       
    I = [m*eye(3), zeros(3); ... %Pose Inertial Tensor
         zeros(3),       IJ];
     
end

