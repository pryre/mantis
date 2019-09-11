function [ eRw, ew, ewd ] = error_R_w_wd_trans(  R_sp, R, w_sp, w, wd_sp )
%ERROR_W_TRANS Summary of this function goes here
%   Detailed explanation goes here

    % Error between current and setpoint rotations (R -> Rsp)
    eR = R'*R_sp;

    % eRw is the rotation error (as an omega) in the body frame at the
    % current time.
    % eRw = 0.5*vee_down(R'*R_sp - R_sp'*R); %XXX: Textbook notation
    eRw = 0.5*vee_down(eR - eR');

    % ew is the omega error in the body frame at the current time.
    ew = eR*w_sp - w;
    
    % ewd tracking acceleration omega (not really error) in the body frame
    % at the current time. The second term here is left over from the
    % calculation of eRd, where it splits into two parts and the second
    % part cancels due to a (w_sp^)*w_sp term.
    ewd = eR*wd_sp - vee_up(w)*eR*w_sp;

end

