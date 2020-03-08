% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ gi ] = inverse_trans( g )
%INVERSE_TRANS Summary of this function goes here
%   Detailed explanation goes here

    gp = g(1:3,4);
    gR = g(1:3,1:3);
    gi = [       gR', -gR'*gp; ...
          zeros(1,3),       1];
end

