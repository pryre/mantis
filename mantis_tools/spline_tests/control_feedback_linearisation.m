function [ tau ] = control_feedback_linearisation( masses, n, r, xd, xdd_sp )
%COMPUTED_TORQUE_CONTROL Summary of this function goes here
%   Detailed explanation goes here

    ANGULAR = 1:3;
    LINEAR = 4:6;
    JOINTS = 7:7+n-1;
    
    [ Dq, Cqqd ] = calc_mass_properties( masses, n, r, xd );

    % The body-fixed coriolis terms need to be selectively removed. We do
    % not want the coriolis effects from changes in angular momentum, but
    % we rely on those that effect linear momentum (as part of the 
    % requirement for differential flatness.
%     x_c = [x(ANGULAR);0;0;0];
    tau = Dq*xdd_sp + Cqqd*xd;
end

