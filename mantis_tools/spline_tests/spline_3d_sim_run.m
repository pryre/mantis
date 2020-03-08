% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [dx] = spline_3d_sim_run(x,tau,mass)
%PCT_RUN Summary of this function goes here
%   Detailed explanation goes here
% x = [zeros(3,1);    % Angular Velocity (Body)
%      zeros(3,1)];   % Linear Velocity (Body)
% v = zeros(size(x)); % Body frame acclerations
% mass_c % mass as seen by the controller
% mass_d % mass as seen by the dynamics
  
    % Variables
    dx = zeros(size(x));
%     g = [0;0;-9.80665];
    ANGULAR = 1:3;
    LINEAR = 4:6;
    
%     Dq_c = inertial_gen(mass_c.Ixx,mass_c.Iyy,mass_c.Izz,mass_c.m);
%     Cqqd_c = [cross(x(ANGULAR),diag([mass_c.Ixx,mass_c.Iyy,mass_c.Izz])*x(ANGULAR));0;0;0];
%     Dq_d = inertial_gen(mass_d.Ixx,mass_d.Iyy,mass_d.Izz,mass_d.m);
%     Cqqd_d = [cross(x(ANGULAR),diag([mass_d.Ixx,mass_d.Iyy,mass_d.Izz])*x(ANGULAR));0;0;0];
    
    M = inertial_gen(mass.Ixx,mass.Iyy,mass.Izz,mass.m);
   
    Dq_d = M;
    Cqqd_d = [vee_up(x(ANGULAR)),zeros(3,3);zeros(3,3),vee_up(x(ANGULAR))]*M;
%     Cqqd_d = [vee_up(x(ANGULAR)),zeros(3,3);zeros(3,3),zeros(3,3)]*M;

    % Calculate force input (force/torque) using computed torque control
    %     tau = [mass.Ixx*v(4);    % X-axis rotational acceleration
    %            mass.Iyy*v(5);    % Y-axis rotational acceleration
    %            mass.Izz*v(6);    % Z-axis rotational acceleration
    %            0;                % X-axis linear acceleration (no X axis thrusters)
    %            0;                % Y-axis linear acceleration (no Y axis thrusters)
    %            mass.m*v(3)];     % Z-axis linear acceleration
    
    dx = inv(Dq_d)*(tau - Cqqd_d*x);
end










