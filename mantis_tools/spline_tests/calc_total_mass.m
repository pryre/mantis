% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ m ] = calc_total_mass( model )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    m = 0;
    for i = 6:model.NB
        m = m + model.I{i}(6,6);
    end
end

