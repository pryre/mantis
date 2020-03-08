% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ Vh ] = cay( V )
%CAY Performs the Cayley map of a SE(3) velocity
%   Detailed explanation goes here

    if size(V) ~= size(zeros(6,1))
        error('Input matrix must be of size 6x1')
    end

    w = V(1:3);
    v = V(4:6);
    w_hat = vee_up(w);
    
    Vh1 = eye(3) + (4/(4 + norm(w)^2))*(w_hat + (w_hat^2)/2);
    Vh2 = (2/(4 + norm(w)^2))*(2*eye(3) + w_hat)*v;
    
    Vh = [Vh1 , Vh2; ...
             zeros(1,3), 1];

end

