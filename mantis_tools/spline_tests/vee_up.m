% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ w_vee ] = vee_up( w )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if size(w) ~= size([0;0;0])
        error('Input vector must be of size 3x1')
    end
    
    w_vee = [    0, -w(3),  w(2); ...
              w(3),     0, -w(1); ...
             -w(2),  w(1),    0];
end

