% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ r ] = thrustvec_angle_error(z_sp,z)
%THRUSTVEC_ANGLE_ERROR Calculates the angle error between pairs of
%normalised thrust vectors.
    EPS = 1e-5;

    if size(z_sp,1) ~= 3 || size(z,1) ~= 3
        error('Input vectors must be 3 by X in size')
    end
    
    if size(z_sp,2) ~= size(z,2)
        error('Input vectors must have the same number of columns')
    end
    
    if norm(z_sp(:,1)) - 1 > 2*EPS
        error('Input z_sp must be normalised')
    end
        
    if norm(z(:,1)) - 1 > 2*EPS
        error('Input z must be normalised')
    end
    
    n = size(z_sp,2);
    r = zeros(1,n);
    
    for i = 1:n
        r(i) = acos( dot( z_sp(:,i), z(:,i) ) );
    end
end

