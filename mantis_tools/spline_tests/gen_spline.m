% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ t, s ] = gen_spline( t_vias, vias, order, dt )
%GEN_DATA_POINT_STRUCT Summary of this function goes here
%   Detailed explanation goes here

    % Full trajectory times
    t0 = t_vias(1);
    tf = t_vias(end);
    
    % Pre-allocate vectors
    t = t0:dt:tf;
    s = zeros(5,length(t));
%     t = [];
%     s = zeros(5,0);

    % Itterate over the list of vias in pairs
%     si = 1;

    for i = 2:size(vias,2)
        
        % Segment times
        st0 = t_vias(i-1);
        stf = t_vias(i);
        sdt = stf-st0;
        
%         if mod(sdt,dt)
%             error('Bad alignment between t_vias and dt');
%         end
        
        sr0 = find(t==st0);
        srf = find(t==stf);
        
        if ~any(sr0) || ~any(srf)
            error('Could not align segment times to t');
        end
        
        sr = sr0:srf;
        
        st = linspace(0, 1, length(sr));
        
        
        switch(order)
            case 3
               as = spline_solver_cubic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i));
            case 5
                as = spline_solver_quintic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(3,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i), ...
                                            vias(3,i));
            case 7
                as = spline_solver_septic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(3,i-1), ...
                                            vias(4,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i), ...
                                            vias(3,i), ...
                                            vias(4,i));
            case 9
                as = spline_solver_nonic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(3,i-1), ...
                                            vias(4,i-1), ...
                                            vias(5,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i), ...
                                            vias(3,i), ...
                                            vias(4,i), ...
                                            vias(5,i));
            otherwise
                error('Unknown spline order')
        end


        %% Spline Calculator

        a = zeros(10,1);
        a(1:length(as)) = as;
        a = flip(a)';   % We want decending order (spline_calculator) and horizontal (for polyder)
        sq = spline_calculator(st, sdt, a);
        
        s(:,sr) = sq;
    end
end

