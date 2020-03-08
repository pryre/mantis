% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ Ro ] = orthagonalize_rotm( R )
%NORMALIZE_ROTM Summary of this function goes here
%   Detailed explanation goes here

    Ro = zeros(size(R));

    for i=1:3
        Ro(1:3,i) = R(1:3,i) / norm(R(1:3,i));
    end
end