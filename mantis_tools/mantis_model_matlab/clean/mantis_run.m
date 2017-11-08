function [ yn ] = mantis_run( dt, y, D, C, L, N, varargin )
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
    q = y(1:18);
    v = y(19:26);
    
    g = reshape(q(1:16),[4,4]);
    r = q(17:18);
    V = v(1:6);
    dr = v(7:8);

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
    drn = vn(7:8);
    
    % Step forward to calculate current pose and angles
    rn = r + dt*drn;
    gn = g*cay(dt*Vn);
    
    % Reassemble state
    qn = [gn(:); rn(:)];
    %already have vn

    yn = [qn(:); vn(:)];

%% Old code
%     g0 = reshape(y(1:16),4,4);
%     gd0 = reshape(y(17:32),4,4);
%     r = y(33:34);
%     rd = y(35:36);
%     
%     g0inv = [g0(1:3, 1:3)', -g0(1:3, 1:3)'*g0(1:3, 4); ...
%              zeros(1,3), 1];
%          
%     Vhat = g0inv*gd0;
%     
%     bwx = Vhat(3,2);
%     bwy = Vhat(1,3);
%     bwz = Vhat(2,1);
%     bvx = Vhat(1,4);
%     bvy = Vhat(2,4);
%     bvz = Vhat(3,4);
%          
%     v = [bwx; bwy; bwz; bvx; bvy; bvz; rd(1); rd(2)];
%     disp('v')
%     disp(v')
%     
%     Dq = zeros(size(Dq_eq));
%     Cqqd = zeros(size(Cqqd_eq));
%     N = zeros(size(N_eq));
% 
%     for i = 1:numel(Dq_eq)
%         Dq(i) = Dq_eq{i}(r(1),r(2),bwx,bwy,bwz,bvx,bvy,bvz,rd(1),rd(2));
%     end
%     
%     if isinf(Dq_inv)
%       warn('Dq inf!');
%       %Dq_inv(:) = 0;
%     end
% 
%     for i = 1:numel(Cqqd)
%         Cqqd(i) = Cqqd_eq{i}(r(1),r(2),bwx,bwy,bwz,bvx,bvy,bvz,rd(1),rd(2));
%     end
%     
%     for i = 1:numel(N_eq)
%         N(i) = N_eq{i}(g0(1),g0(2),g0(3),g0(4),g0(5),g0(6),g0(7),g0(8),g0(9),g0(10),g0(11),g0(12),g0(13),g0(14),g0(15),g0(16),r(1),r(2));
%     end
%     
%     if do_control == 1
%         error('Not just yet!')
%         u = Dq*v + (Cqqd + Kqd)*qd + phi;
%     else
%         u = zeros(8,1);
%     end
%     
%     disp('N')
%     disp(N)
%     Dq_inv(inv(Dq_inv)==Inf) = 0;
%     
%     accel = -(inv(Dq)*(Cqqd*qd + Kqd*qd + phi)) + inv(Dq)*u_sub;
%     accel = -(Dq\((Cqqd + Kqd)*v + N)) + Dq\u;
%     disp('accel')
%     disp(accel')
%     
%     if isnan(accel)
%        error('NaN'); 
%     end
%     
%     Convery back to world frame
%     dwhat = [0, -accel(3), accel(2); ...
%              accel(3), 0, -accel(1); ...
%              -accel(2), accel(1), 0];
% 
%     dVhat = [dwhat, [accel(4); accel(5); accel(6)]; ... % Velocity in ineratial frame
%              zeros(1,3), 0]
%     
%     dgd0 = g0*dVhat;
%     
%     drd = [accel(7); accel(8)];
%     
%     dy(1) = y(2);
%     dy(2) = accel(1);
%     dy(3) = y(4);
%     dy(4) = accel(2);
%     
%     dy = [gd0(:);
%           dgd0(:);
%           rd;
%           drd];
%       
%     disp('dy')
%     disp(dy')
%     
end