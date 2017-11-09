function [ yn ] = multirotor_run( dt, y, D, C, L, N, varargin )
%MANTIS_RUN Runs a simulation for the Mantis MM-UAV
%   Description
%   Additional Arguments:
%       v: acceleration input in body frame
    
    %======================
    % dv = (vn - vk)/dt
    % Bu = M(rk)*dv + b(qk,vk)
    %
    % gn = gk*cay(dt*Vn)
    % rn = rk*dt*drn
    %======================

    %% Prep current state
    % get current state y(q,v)
    % Pose, arm joints, and body-frame velocities
    q = y(1:16);
    v = y(17:22);
    
    g = reshape(q(1:16),[4,4]);
    V = v(1:6);

    %% Control
    %Check for control input or just free response
    if isempty(varargin)
        ci = zeros(size(D,2),1);
    else
       % Constant control matrix
       B = varargin{1};
       
       % Goal body frame accelerations
       a = varargin{2};
       
       % Calculate control input here
       % ci = B*u;
    end
    
    %% Calculate response
    % Calculate acceleration (dv) from dynamics
    % This is semi-implicity due to the propegation
    % M(rk)*dv + b(qk,vk) = Bu
    dv = -(D\((C + L)*v + N)) + D\ci;
    
    % Integrate acceleration to get next body velocity
    vn = v + dv*dt;

    Vn = vn(1:6);
    
    % Step forward to calculate current pose and angles
    gn = g*cay(dt*Vn);
    
    % Reassemble state
    qn = gn(:);
    %already have vn

    yn = [qn(:); vn(:)];

end