% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [X_be, eta_be, etad_be] = FKfly(model, n, q, qd, qdd)
%FKfly_serial Calculates the forward kinematics for each link until joint n
%   Inputs:
%       - model: spacial_v2 model to use
%       -     q: joints states
%       -     n: link number to calculate end of

%     sn = state_names_lookup(model.NB - 6);

%     r = x(sn.STATE_R);
    
    % Skim the kinematic tree to find solution of the chain to link n
    p = n + 6;
    link_list = p;
    while(p > 6)
        p = model.parent(p);
        link_list = [p, link_list];
    end
    
    % Drop the base link from the chain (start in base frame of reference)
    link_list(link_list==6) = [];
    qe = [q; 0]; % Add a phantom link joint for the end effector
    qde = [qd; 0]; % Add a phantom link joint for the end effector
    qdde = [qdd; 0]; % Add a phantom link joint for the end effector
    X_be = eye(6);
    eta_be = [];
    etad_be = [];
    if(length(qd) == length(q))
        eta_be = zeros(6,1);
        
        if(length(qdd) == length(q))
            etad_be = zeros(6,1);
        end
    end
    
    for i = link_list
        j = i-6;
        [ Xj, S ] = jcalc( model.jtype{i}, qe(j) );
        Xup = Xj*model.Xtree{i};
        X_be = Xup*X_be;
        
        if length(qd) == length(q)
            vJ = S*qde(j);
            eta_be = Xup*eta_be + vJ;
            
            if(length(qdd) == length(q))
                etad_be = Xup*etad_be + S*qdde(j) + crm(eta_be)*vJ;
            end
        end
        
%         g_j = inverse_trans(pluho(Xup));
%         g_be = g_be*g_j;
    end
%     
%     
% 
%     g_b = [quat2rotm(x(sn.STATE_Q)'), x(sn.STATE_XYZ);
%            zeros(1,3), 1];
%     g_e = g_b*g_be;
end

