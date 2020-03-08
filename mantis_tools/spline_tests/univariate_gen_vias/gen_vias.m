% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ vias ] = gen_vias( varargin )
%GEN_VIAS_LINEAR Summary of this function goes here
%   Detailed explanation goes here
    
    if nargin < 3
        error(['Not enough arguments specified (' num2str(nargin), ')'])
    end
    
    [method, positions, seg_duration] = varargin{1:3};
    conditioning = 'Unconditioned';
    
    if nargin > 3
        for i = 4:nargin
            key = varargin{i};
            
            if i+1 <= nargin && ischar(key)
                val = varargin{i+1};

                if strcmp(key, 'Conditioning')
                    conditioning = val;
                end
            end
        end
    end
    
    vias = zeros(5,length(positions));
    vias(1,:) = positions;
    f = @(str)[];
    
    if strcmp(method, 'linear')
        vias(2,:) = est_dvia_linear(vias(1,:), seg_duration, conditioning);
        vias(3,:) = est_dvia_linear(vias(2,:), seg_duration, conditioning);
        vias(4,:) = est_dvia_linear(vias(3,:), seg_duration, conditioning);
        vias(5,:) = est_dvia_linear(vias(4,:), seg_duration, conditioning);
    elseif strcmp(method, 'fdcc')
        vias = est_dvia_fdcc(vias(1,:), seg_duration, conditioning);
    else
        error(['Unknown method: ', method])
    end
    
end

