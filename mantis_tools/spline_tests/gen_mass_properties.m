% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ masses ] = gen_mass_properties( n )
%GEN_MASS_PROPERTIES Generates preset mass properties for system bodies
% Detailed desctription

    JOINT_MASS = 0.2;
    JOINT_IXX = 0.2;
    JOINT_IYY = 0.2;
    JOINT_IZZ = 0.2;

    masses.base.m = 1.83497;
    masses.base.Ixx = 0.02961;
    masses.base.Iyy = 0.02961;
    masses.base.Izz = 0.05342;
    
    for i = 1:n
        masses.(['link_', num2str(n)]).m = JOINT_MASS;
        masses.(['link_', num2str(n)]).Ixx = JOINT_IXX;
        masses.(['link_', num2str(n)]).Iyy = JOINT_IYY;
        masses.(['link_', num2str(n)]).Izz = JOINT_IZZ;
    end
end
