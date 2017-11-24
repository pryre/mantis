function [ yn ] = multirotor_run( dt, y, M, N, G, F, A, varargin )
%MANTIS_RUN Runs a simulation for the Mantis MM-UAV
%   Description
%   Additional Arguments:
%       v: acceleration input in body frame
    
%%

    %Check for control input or just free response
    ci = zeros(size(M,2),1);
        
    if ~isempty(varargin)
       % Constant control matrix
       B = varargin{1};
       
       % Goal body frame accelerations
       a = varargin{2};
       
       % Calculate control input here
       % ci = B*u;
    end
    
    
    ysplit = reshape(y, [length(y)/2,2]);
    q = ysplit(:,1);
    qd = ysplit(:,2);
    
    
    % A(M*qdd + N*qd) = A(M*G + U + F)
    %RHS = A*(M*G + ci + F);
    %qdd = M\(A\RHS - N*qd);
    RHS = M*G + ci + F;
    qdd = M\(RHS - N*qd);
    
    qdn = qd + dt*qdd;
    qn = q + dt*qdn;
    
    
    yn = [qn(:); qdn(:)];
    


%% Old

%     %======================
%     % dv = (vn - vk)/dt
%     % Bu = M(rk)*dv + b(qk,vk)
%     %
%     % gn = gk*cay(dt*Vn)
%     % rn = rk*dt*drn
%     %======================
% 
%     %% Prep current state
%     % get current state y(q,v)
%     % Pose, arm joints, and body-frame velocities
%     q = y(1:16);
%     v = y(17:22);
%     
%     g = reshape(q(1:16),[4,4]);
%     V = v(1:6);
% 
%     %% Control
%     %Check for control input or just free response
%     ci = zeros(size(D,2),1);
%         
%     if ~isempty(varargin)
%        % Constant control matrix
%        B = varargin{1};
%        
%        % Goal body frame accelerations
%        a = varargin{2};
%        
%        % Calculate control input here
%        % ci = B*u;
%     end
%     
%     %% Calculate response
%     % Calculate acceleration (dv) from dynamics
%     % This is semi-implicity due to the propegation
%     % M(rk)*dv + b(qk,vk) = Bu
%     dv = -(D\((C + L)*v + N)) + D\ci;
%     
%     % Integrate acceleration to get next body velocity
%     vn = v + dv*dt;
%     Vn = vn(1:6);
%     
%     % Step forward to calculate current pose and angles
%     gn = g*cay(dt*Vn);
%     
%     % Reassemble state
%     qn = gn(:);
%     %already have vn
% 
%     yn = [qn(:); vn(:)];

end