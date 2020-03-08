% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ body ] = add_body_link( body, name, type, mass, center_of_mass, rotation, dim )
%ADD_BODY_LINK Summary of this function goes here
%   Detailed explanation goes here

    body(end+1,:) = {name, type, mass, center_of_mass, rotation, dim, zeros(3,3)};

end

