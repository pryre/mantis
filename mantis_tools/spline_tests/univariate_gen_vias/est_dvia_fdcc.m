% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [vias] = est_dvia_fdcc(v, dt, conditioning)
%LINEAR_EST Summary of this function goes here
%   Detailed explanation goes here

    EPS = 1e-5;

    np = length(v);
    vias = zeros(5,np);

    kernel_l = 3;
    kernel_r = 3;
    kernel = -kernel_l:kernel_r;

    vias(1,:) = v;

    % If otherwise, it is a linear segment

    for i = 2:np-1
        kt = kernel( kernel+i > 0 & kernel+i <= np );

        qp = vias(1,i-1);
        qc = vias(1,i);
        qn = vias(1,i+1);

        cond_holding = abs(qc - qp) < EPS || abs(qc - qn) < EPS;
        cond_maxima = (qc > qp) && (qc > qn);
        cond_minima = (qc < qp) && (qc < qn);

        cond_isexact = strcmp(conditioning, 'Exact');
        cond_isholding = strcmp(conditioning, 'Holding') && cond_holding;
        cond_isprecise = strcmp(conditioning, 'Precise') && ( cond_holding || cond_maxima || cond_minima);

%         if false
%         if (qc == qp) || (qc == qn)
%         if (qc == qp) || (qc == qn) || ( (qc < qp) && (qc < qn) ) || ( (qc > qp) && (qc > qn) )
            % Via point conditioning for precise response:
            %   Hold: (qc ~ qp) || (qc ~ qn)
            %   Minima: (qc < qp) && (qc < qn)
            %   Maxima: (qc > qp) && (qc > qn)
        if cond_isexact || cond_isholding || cond_isprecise
            vias(2:end,i) = 0;
        else
            for order = 1:4
                if length(kt) <= order
                    % This is to protect against a list of vias that is too
                    % short to accurately estimate at the appropriate order
                    vias(order+1,i) = 0;
                else
                    points = vias(1,kt + i);
                    coeffs = fdcc(kt,order);
                    vias(order+1,i) = (points*coeffs)/(dt^(order));
                end
            end
        end
    end
end

