% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [dvias] = est_dvia_linear(vias, dt, conditioning)
%LINEAR_EST Summary of this function goes here
%   Detailed explanation goes here

    dvias = zeros(size(vias));
    
    % If otherwise, it is a linear segment
    if length(vias) > 2
        for i = 2:length(vias)-1
            qp = vias(i-1);
            qc = vias(i); 
            qn = vias(i+1);   
            
            cond_holding = abs(qc - qp) < EPS || abs(qc - qn) < EPS;
            cond_maxima = (qc > qp) && (qc > qn);
            cond_minima = (qc < qp) && (qc < qn);
            
            cond_isexact = strcmp(conditioning, 'Exact');
            cond_isholding = strcmp(conditioning, 'Holding') && cond_holding;
            cond_isprecise = strcmp(conditioning, 'Precise') && ( cond_holding || cond_maxima || cond_minima);
            
%             dvias(i) = (qn - qp)/(2*dt);
            
%             if (qc == qp) || (qc == qn) || ( (qc < qp) && (qc < qn) ) || ( (qc > qp) && (qc > qn) )
%                 dvias(i) = 0;
            if cond_isexact || cond_isholding || cond_isprecise
                dvias(i) = 0;
            else
                dvias(i) = (qn - qp)/(2*dt);
            end
        end
    end
end

