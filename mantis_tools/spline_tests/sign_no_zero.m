% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ s ] = sign_no_zero( x )
%SIGN_NO_ZERO Summary of this function goes here
%   Detailed explanation goes here

    s = (zeros(size(x)) <= x) - (x < zeros(size(x)));
end

