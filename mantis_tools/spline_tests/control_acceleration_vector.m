% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ vd_b, acc_c, R_c, q_c, integrator ] = control_acceleration_vector(pos_sp, vel_sp, acc_sp, yaw_sp, pos, vel, R, Kp, Kd, Ki, integrator, dt)
%CONTROL_ACCELERATION_VECTOR Summary of this function goes here
%   Detailed explanation goes here

    %% PD Tracking Controller
    
            
    % Common integrator for all methods

    
    % Build acceleration control reference from spline tracking error
    ep = (pos_sp - pos);
    ev = (vel_sp - vel);
    
    if Ki > 0
        integrator = integrator + ep*dt;
    end
    
    acc_c = acc_sp + Kp*ep + Ki*integrator + Kd*ev;

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

