% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ I ] = calc_sphere( mass, dim )
%CALC_SPHERE Summary of this function goes here
%   Detailed explanation goes here

    assert(isequal(size(dim), [1,1]), 'Sphere dim must be: [radius]')

    I = zeros(3,3);

	Ixyz = (2*mass/5)*(dim*dim);
    I(1,1) = Ixyz;
    I(2,2) = Ixyz;
    I(3,3) = Ixyz;
end

