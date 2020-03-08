% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ Dq, Cqqd ] = calc_mass_properties( masses, n, r, xd )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    ANGULAR = 1:3;
    LINEAR = 4:6;
    JOINTS = 7:7+n-1;
    
    Dq = zeros(length(xd),length(xd));
    Cqqd = zeros(size(Dq));
end

