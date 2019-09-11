function [ vd_b, acc_c, R_c, q_c ] = control_acceleration_vector(pos_sp, vel_sp, acc_sp, yaw_sp, pos, vel, R, Kp, Kd)
%CONTROL_ACCELERATION_VECTOR Summary of this function goes here
%   Detailed explanation goes here

    %% PD Tracking Controller
    % Build acceleration control reference from spline tracking error
    acc_c = acc_sp + Kp*(pos_sp - pos) + Kd*(vel_sp - vel);

    % Rotation control reference from acceleration vector with tracking error
    [R_c, q_c] = rot_from_vec_yaw(acc_c, yaw_sp);

    
    %% Directional Thrust Vectoring
    z_acc = norm(acc_c);

    % Calculate cos(theta) difference of z basis to acceleration vector
    %
    % Other options would be to set the value to zero over a set
    % cut-off (e.g. pi/8) which would produce a stepping effect.
    % Similarly, the dot() (or cos()) method could be dot()^2 to be more agresssive
    % (expotential?) in the trail-off
    thrust_scale = dot(R_c(:,3),R(:,3));

    % Inversion protection
    if (z_acc <= 0) || (thrust_scale <= 0)
    % warning('Large thrust difference or reverse thrust detected')
        thrust_scale = 0;
    end

    % Compile Reference Body Accelerations
    vd_b = [0;0;thrust_scale*z_acc];% - vee_up(x(STATE_WXYZ_B,i-1))*x(STATE_VXYZ_B,i-1);

end

