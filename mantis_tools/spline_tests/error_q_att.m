function [ eq ] = error_q_att( q_sp, q, yaw_w )
%ERROR_Q_ATT Summary of this function goes here
%   Detailed explanation goes here

    EPS = 1e-5;

    R_sp = quat2rotm(q_sp);
    R = quat2rotm(q);
    z_sp = R_sp(1:3,3);
    z = R(1:3,3);

    q_sp_red = rotm2quat(vrrotvec2mat(vrrotvec(z, z_sp)));
    q_sp_red_w = [1,0,0,0];

    if (abs(q_sp_red(2)) > (1 - EPS) || abs(q_sp_red(3)) > (1 - EPS))
        % In the infinitesimal corner case where the vehicle and thrust 
        % have the completely opposite direction, full attitude control
        % anyways generates no yaw input and directly takes the combination
        % of roll and pitch leading to the correct desired yaw. Ignoring 
        % this case would still be totally safe and stable.

       % warning('Full rotation!')

        q_sp_red_w = q_sp;
    else
        % transform rotation from current to desired thrust vector into a 
        % world frame reduced desired attitude
        q_sp_red_w = quatmultiply(q_sp_red, q);
    end

    
    % mix full and reduced desired attitude
    q_mix = quatmultiply(quatinv(q_sp_red_w), q_sp);
    % Correct for inversed direction of rotation (if q.w is negative)
    q_mix = sign_no_zero(q_mix(1)).*q_mix;
    % catch numerical problems with the domain of acosf and asinf
%     q_mix(1) = math::constrain(q_mix(1), -1.f, 1.f);
%     q_mix(4) = math::constrain(q_mix(4), -1.f, 1.f);
    qy_w = [cos(yaw_w * acos(q_mix(1))),0,0,sin(yaw_w * asin(q_mix(4)))];
    qd = quatmultiply(q_sp_red_w, qy_w);

    % quaternion attitude control law, qe is rotation from q to qd
    qe = quatmultiply(quatinv(q), qd);

    % using sin(alpha/2) scaled rotation axis as attitude error (see 
    % quaternion definition by axis angle) also taking care of the 
    % antipodal unit quaternion ambiguity
    eq = (2 * sign_no_zero(qe(1)) * qe(2:4))'; %Transpose to get [x;y;z]

%     rate_setpoint = eq.*Kp;
end

