% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ tau, acc_c, q_c, ang_integrator ] = control_nonlinear_pid(pos_sp, vel_sp, acc_sp, yaw_sp, w_sp, x, KxP, KxdP, KtP, KtdP, yaw_w, dt, ang_integrator)
%CONTROL_NONLINEAR_PID Summary of this function goes here
%   Detailed explanation goes here

    % Gain Scaling
    % This why we pick gains for MC_RATE_P, etc., to be 50-100x larger than
    % the ones used in the PX4 firmware.
    gs = 1/50;
            
    % Instead of normalising the thrust vector, simply use F=ma
    kT = calc_total_mass(model);
    
    sn = state_names_lookup(model.NB-6);
    R = quat2rotm(x(sn.STATE_Q)');

    %% High-Level 
    [ vd_b, acc_c, ~, q_c ] = control_acceleration_vector(pos_sp, vel_sp, acc_sp, yaw_sp, ...       % References
                                                          x(sn.STATE_XYZ), x(sn.STATE_VXYZ), R, ... % States
                                                          KxP, KxdP);                               % Gains

    %% Low-Level
    

    Ki = KtP / 5;
    
    eR = error_q_att(q_c, x(sn.STATE_Q)', yaw_w);
    ew = w_sp - x(sn.STATE_WXYZ_B);
    
    ang_integrator = ang_integrator + eR*dt;
    
    wd_b = KtP*eR + KtdP*ew + Ki*ang_integrator;
    
    
    %% Gain Reduction Factor
    % Don't use feedback linearisation, instead we simply use a term to
    % scale the gains such that they can be tuned in a nicer fashion.
    %     xid_b = [wd_b;
    %              vd_b];
    
    tau = [gs*wd_b; ...
           kT*vd_b];

end

