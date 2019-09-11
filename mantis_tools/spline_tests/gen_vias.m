function [ vias ] = gen_vias( varargin )
%GEN_VIAS_LINEAR Summary of this function goes here
%   Detailed explanation goes here
    
    if nargin < 3
        error(['Not enough arguments specified (' num2str(nargin), ')'])
    end
    
    [method, positions, tv] = varargin{1:3};
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
    
    if strcmp(method, 'fdcc')
        vias = est_dvia_fdcc(positions, tv, conditioning);
    else
        error(['Unknown method: ', method])
    end
    
end

