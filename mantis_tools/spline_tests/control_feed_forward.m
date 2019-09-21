function [ tau, acc_c, q_c, integrator ] = control_feed_forward(model, pos_sp, vel_sp, acc_sp, yaw_sp, x, x_p, dt, integrator)
%CONTROL_FEED_FORWARD Summary of this function goes here
%   Detailed explanation goes here

    % Hardcode gains here, as they do in PX4
    KxP = 1;
    KxdP = 2;
    MC_ANG_P = 4.5;
    MC_RATE_P = 7.5;
    MC_RATE_I = 2.5;
    MC_RATE_D = 0.15;
    yaw_w = 0.6;

    % Gain Scaling
    % This why we pick gains for MC_RATE_P, etc., to be 50-100x larger than
    % the ones used in the PX4 firmware.
    gs = 1/30;
            
    % Instead of normalising the thrust vector, simply use F=ma
    kT = model.I{6}(6,6);
    
    sn = state_names_lookup(model.NB-6);
    R = quat2rotm(x(sn.STATE_Q)');

    %% High-Level 
    [ vd_b, acc_c, ~, q_c ] = control_acceleration_vector(pos_sp, vel_sp, acc_sp, yaw_sp, ...       % References
                                                          x(sn.STATE_XYZ), x(sn.STATE_VXYZ), R, ... % States
                                                          KxP, KxdP);                               % Gains


    %% Low-Level
    w_sp = MC_ANG_P*error_q_att(q_c, x(sn.STATE_Q)', yaw_w);
    wd_sp = zeros(3,1);

    ew = w_sp - x(sn.STATE_WXYZ_B);
    ewd = wd_sp - (x(sn.STATE_WXYZ_B) - x_p(sn.STATE_WXYZ_B))/dt;

    integrator = integrator + ew*dt;

    wd_b = MC_RATE_P*ew + MC_RATE_I*integrator + MC_RATE_D*ewd;
    
    
    %% Gain Reduction Factor
    % Don't use feedback linearisation, instead we simply use a term to
    % scale the gains such that they can be tuned in a nicer fashion.
    %     xid_b = [wd_b;
    %              vd_b];
    
    tau_pid = [gs*wd_b; ...
               kT*vd_b];
     

    %% Feedback Linearisation
    xid_b = [0;0;0;
             0;0;9.80665];
    rdd = zeros(model.NB-6,1); % No access to the low-level servo commands

    [tau_f, ~] = IDfly( model, x(sn.STATE_XI_B), xid_b, x(sn.STATE_R), x(sn.STATE_RD), rdd );

    tau_ff = tau_f - kT*xid_b; % Remove gravity thrust
    
    %% Control Output
    % PID output plus feedforward terms
    tau = tau_pid + tau_ff;
    
    
end

