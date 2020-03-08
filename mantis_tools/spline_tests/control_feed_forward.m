% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ tau, acc_c, q_c, pos_integrator, w_integrator ] = control_feed_forward(model, ...
                                                            pos_sp, vel_sp, acc_sp,...
                                                            yaw_sp,...
                                                            x, x_p,...
                                                            dt, pos_integrator, w_integrator,...
                                                            KxP, KxI, KxdP)
%CONTROL_FEED_FORWARD Summary of this function goes here
%   Detailed explanation goes here

    % Hardcode gains here, as they do in PX4
    MC_ANG_P = 4.5;
    MC_RATE_P = 15.0;
    MC_RATE_I = 2.5;
    MC_RATE_D = 0.15;
    yaw_w = 0.6;

    % Gain Scaling
    % This why we pick gains for MC_RATE_P, etc., to be 50-100x larger than
    % the ones used in the PX4 firmware.
    gs = 1/20;
            
    % Instead of normalising the thrust vector, simply use F=ma
    % In this case, we use just the base mass, as the rest is added later
    kT = model.I{6}(6,6);
    
    sn = state_names_lookup(model.NB-6);
    R = quat2rotm(x(sn.STATE_Q)');

    %% High-Level 
    [ vd_b, acc_c, ~, q_c, pos_integrator] = control_acceleration_vector(pos_sp, vel_sp, acc_sp, yaw_sp, ...       % References
                                                          x(sn.STATE_XYZ), x(sn.STATE_VXYZ), R, ... % States
                                                          KxP, KxdP, ... % Gains
                                                          KxI, pos_integrator, dt); %Integral Terms


    %% Low-Level
    w_sp = MC_ANG_P*error_q_att(q_c, x(sn.STATE_Q)', yaw_w);
    wd_sp = zeros(3,1);

    ew = w_sp - x(sn.STATE_WXYZ_B);
    ewd = wd_sp - (x(sn.STATE_WXYZ_B) - x_p(sn.STATE_WXYZ_B))/dt;

    w_integrator = w_integrator + ew*dt;

    wd_b = MC_RATE_P*ew + MC_RATE_I*w_integrator + MC_RATE_D*ewd;
    
    
    %% Gain Reduction Factor
    % Don't use feedback linearisation, instead we simply use a term to
    % scale the gains such that they can be tuned in a nicer fashion.
    %     xid_b = [wd_b;
    %              vd_b];
    
    tau_pid = [gs*wd_b; ...
               kT*vd_b];
     

    %% Feedforward  Linearisation
    
    % Set the body to accelerate vertically if the high-level is applying
    % thrust as well. This is our "steady" condition (stable hover)
    t_switch = double(vd_b(3) > 0);
    xid_b = [0;0;0;0;0;t_switch*9.80665];
    rdd = zeros(model.NB-6,1); % No access to the low-level servo commands

    [tau_f, ~] = IDfly( model, x(sn.STATE_XI_B), xid_b, x(sn.STATE_R), x(sn.STATE_RD), rdd );

    % Remove gravity thrust of this calc as it comes from the PID controllers
    tau_ff = tau_f - kT*xid_b;
    
    
    %% Control Output
    % PID output plus feedforward terms
    tau = tau_pid + tau_ff;
    
    
end

