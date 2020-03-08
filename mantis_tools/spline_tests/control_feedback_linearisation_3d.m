% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ tau ] = control_feedback_linearisation_3d(mass,x,xd_sp)
%COMPUTED_TORQUE_CONTROL Summary of this function goes here
%   Detailed explanation goes here

    ANGULAR = 1:3;
    LINEAR = 4:6;
    
    M = inertial_gen(mass.Ixx,mass.Iyy,mass.Izz,mass.m);
%     Cqqd = [vee_up(x(ANGULAR)),zeros(3,3);zeros(3,3),vee_up(x(ANGULAR))]*M;
    Cqqd = [vee_up(x(ANGULAR)),zeros(3,3);zeros(3,3),zeros(3,3)]*M;
    Dq = M;

    % The body-fixed coriolis terms need to be selectively removed. We do
    % not want the coriolis effects from changes in angular momentum, but
    % we rely on those that effect linear momentum (as part of the 
    % requirement for differential flatness.
%     x_c = [x(ANGULAR);0;0;0];
    tau = Dq*xd_sp + Cqqd*x;

end

