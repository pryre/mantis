% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ tau_r, integrator ] = control_manip_decoupled(model, r_sp, rd_sp, x, dt, integrator )
%COMPUTED_TORQUE_CONTROL Summary of this function goes here
%   Detailed explanation goes here

    sn = state_names_lookup(model.NB-6);

%     KrP = 1.25;
%     KrdP = 0.35;
%     KrdI = 1.8;
    KrP = 1.25;
    KrdP = 0.1;
    KrdI = 1.8;
    
    %% High-Level 
    
    er = r_sp - x(sn.STATE_R);
    erd = KrP*er + rd_sp - x(sn.STATE_RD);

    integrator = integrator + erd*dt;

    tau_r = KrdP*erd + KrdI*integrator;
    
    
end

