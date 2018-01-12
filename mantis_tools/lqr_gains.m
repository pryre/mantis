% Script to exploit LQR gains for use as a simple high-level
% position and velocity controller

% Single state matrix
%    x = [p     % position
%         v];   % velocity
%
%   xd = [v;    % velocity
%         a];   % acceleration
a = [0, 1;
     0, 0];
b = [0
     1];

% Full State
% xp = [px   % position X
%       py   % position Y
%       pz   % position Z
%
% xw = [wx   % rotation X
%       wy   % rotation Y
%       wz]; % rotation Z
%
% xr = [r1   % rotation r1
%       r2   % rotation r2

% Position
Ap = a;
Bp = b;
Rp = 0.05;
Qp = [1,  0;
      0, 1];

kp = lqr(Ap,Bp,Qp,Rp);
Kp = repmat(kp,1,3);

% Rotation
Aw = a;
Bw = b;
Rw = 0.005;
Qw = [1,  0;
      0, 2];

kw = lqr(Aw,Bw,Qw,Rw);
Kw = repmat(kw,1,3);

% Manipulator
Ar = a;
Br = b;
Rr = 0.01;
Qr = [1,  0;
      0, 10];

kr = lqr(Ar,Br,Qr,Rr);
Kr = repmat(kr,1,2);

disp('Gain matrix (Kp):')
disp(Kp)
disp('Gain matrix (Kw):')
disp(Kw)
disp('Gain matrix (Kr):')
disp(Kr)







