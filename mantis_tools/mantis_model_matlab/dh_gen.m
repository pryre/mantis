% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ g ] = dh_gen( d, theta, r, alpha )
%DH_GEN Summary of this function goes here
%   Detailed explanation goes here

    %Translation of d
    Td = [1, 0, 0, 0; ...
          0, 1, 0, 0; ...
          0, 0, 1, d; ...
          0, 0, 0, 1];

    %Rotation of theta
    Rt = [cos(theta), -sin(theta), 0, 0; ...
          sin(theta),  cos(theta), 0, 0; ...
                   0,           0, 1, 0; ...
                   0,           0, 0, 1];

    if isa(theta,'double')
        Rt(abs(Rt)<eps) = 0;
    end

    %Translation of r
    Tr = [1, 0, 0, r; ...
          0, 1, 0, 0; ...
          0, 0, 1, 0; ...
          0, 0, 0, 1];

    %Rotation of alpha
    Ra = [1,          0,           0, 0; ...
          0, cos(alpha), -sin(alpha), 0; ...
          0, sin(alpha),  cos(alpha), 0; ...
          0,          0,           0, 1];

    if isa(alpha,'double')
        Ra(abs(Ra)<eps) = 0;
    end

    %Formulate transformation
    g = Rt*Td*Tr*Ra;

end

