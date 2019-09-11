function [ tau, acc_c, q_c ] = control_computed_torque( model, pos_sp, vel_sp, acc_sp, yaw_sp, R_sp, w_sp, wd_sp, r_sp, rd_sp, rdd_sp, x, KxP, KxdP, KtP, KtdP, yaw_w, KrP, KrD )
%COMPUTED_TORQUE_CONTROL Summary of this function goes here
%   Detailed explanation goes here

    sn = state_names_lookup(model.NB-6);
    R = quat2rotm(x(sn.STATE_Q)');


    %% High-Level 
    [ vd_b, acc_c, ~, q_c ] = control_acceleration_vector(pos_sp, vel_sp, acc_sp, yaw_sp, ...       % References
                                                            x(sn.STATE_XYZ), x(sn.STATE_VXYZ), R, ... % States
                                                            KxP, KxdP);                               % Gains

    
    %% Low-Level
    % Calculate the mapping for body-rate setpoints
    [~, ew, ewd] = error_R_w_wd_trans(R_sp, R, ...
                                      w_sp, x(sn.STATE_WXYZ_B), ...
                                      wd_sp);

    % Calculate a different attitude rotation error function for the
    % that applies the control for the position/velocity terms.
    % XXX: This won't work at inversion singularity (i.e R_p = -R_sp_c)
    eR = error_q_att(q_c, x(sn.STATE_Q)', yaw_w);
%     eR = 0.5*vee_down(R'*R_c - R_c'*R);

    % SO(3) PD Tracking
    wd_b = ewd ...
         + KtdP*ew ...
         + KtP*eR;
     

    %% Feedback Linearisation
    xid_b = [wd_b;
             vd_b];

    er = r_sp - x(sn.STATE_R);
    erd = rd_sp - x(sn.STATE_RD);
    rdd = rdd_sp + KrD*erd + KrP*er;
         
%     tau = control_feedback_linearisation(masses, x(sn.STATE_R), x(sn.STATE_REDUCED), xid_b);

    % The function IDfb() can be used to calculate the forces needed to
    % accelerate each joint, and the accelerations on the robot base that
    % it woul result in
    %   [xdd, tau] = IDfb( model, xfb, x(sn.STATE_R), x(sn.STATE_RD), tau ); 
    % Instead we use the normal inverse dynamics function, however we must
    % be aware of a singularity at q(5)=±pi/2 that exsists due to the method 
    % of calculation used:
    %   "ID itself will work at a singularity, but your ability to specify
    %    a velocity and acceleration of your choice is restricted."
%     q = ... x(sn.STATE_R);
%     qd = ... x(sn.STATE_RD);
%     qdd = ... ...;
    [tauf, taur] = IDfly( model, x(sn.STATE_XI_B), xid_b, x(sn.STATE_R), x(sn.STATE_RD), rdd ); 

    tau = [tauf; ...
           taur];

end
