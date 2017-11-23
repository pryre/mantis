%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
time_start = 0;
time_end = 5;
%time_dt = 0.01;

%g = -9.80665; %m/s

% Load system parameters
params = mantis_params('params.yaml');


%% Generate the system model
% Model is for a locked frame multirotor, such that only it can only pitch,
% but cannot move in any other rotation or translation
%syms l1 l2 lc1 lc2 q1 q2 g m1 m2 Iz1 Iz2 fric 'real'
%syms qd1 qd2 qdd1 qdd2 'real'

syms la l1 l2 g m0 m1 m2 'real'
syms x xd xdd yd ydd z zd zdd 'reat'
syms phi phid phidd thetad thetadd psi psid psidd 'read'
syms r1 r1d r1dd r2 r2d r2dd 'real'


q = [x; y; z; phi; theta; psi; r1; r2];
qd = [xd; yd; zd; phid; thetad; psid; r1d; r2d];
qdd = [xdd; ydd; zdd; phidd; thetadd; psidd; r1dd; r2dd];




Rphi = [1, 0, 0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];
Rtheta = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
Rpsi = [cos(psi), -sin(psi), 0;
        sin(psi), cos(psi), 0;
        0, 0, 1];
    
Rt = Rphi*Rtheta*Rpsi;

warn('fix')
Q = Rt'*ones(3); 

%pd = Rt*pd_body
%w = Rt*w_body

%%

% Formulate D(q)
% Translational Kinetic Energy
disp('Calculating Translational Kinetic Energy')
Jvc1 = [-lc1*sin(q1), 0; ...
        lc1*cos(q1), 0; ...
        0, 0];
Jvc2 = [-l1*sin(q1) - lc2*sin(q1+q2), -lc2*sin(q1+q2); ...
        l1*cos(q1) + lc2*cos(q1+q2), lc2*cos(q1+q2); ...
        0, 0];

tke1 = m1*((Jvc1')*Jvc1);
tke2 = m2*((Jvc2')*Jvc2);

% Rotational Kinetic Energy
disp('Calculating Rotational Kinetic Energy')
rke1 = Iz1*[1, 0; ...
            0, 0];
rke2 = Iz2*[1, 1; ...
            1, 1];

% Total Inertia Matrix
disp('Calculating Inertia Matrix')
tke = tke1 + tke2;
rke = rke1 + rke2;
Dq = tke + rke;

% Cristoffel Symbols
disp('Calculating Cristoffel Symbols')
%cijk= (1/2)*(diff(Dq(k,j), qi) + diff(Dq(k,i), qj) + diff(Dq(i,j), qk));
%cijk=cjik
% %q1
% c111 = (1/2)*(diff(Dq(1,1), q1) + diff(Dq(1,1), q1) - diff(Dq(1,1), q1));
% c121 = (1/2)*(diff(Dq(1,2), q1) + diff(Dq(1,1), q2) - diff(Dq(1,2), q1));
% c211 = c121;
% c221 = (1/2)*(diff(Dq(1,2), q2) + diff(Dq(1,2), q2) - diff(Dq(2,2), q1));
% %q2
% c112 = (1/2)*(diff(Dq(2,1), q1) + diff(Dq(2,1), q1) - diff(Dq(1,1), q2));
% c122 = (1/2)*(diff(Dq(2,2), q1) + diff(Dq(2,1), q2) - diff(Dq(1,2), q2));
% c212 = c122;
% c222 = (1/2)*(diff(Dq(2,2), q2) + diff(Dq(2,2), q2) - diff(Dq(2,2), q2));

%c_col(i,j,k)
% c_sym(1,1,1) = c111;
% c_sym(1,2,1) = c121;
% c_sym(2,1,1) = c211;
% c_sym(2,2,1) = c221;
% c_sym(1,1,2) = c112;
% c_sym(1,2,2) = c122;
% c_sym(2,1,2) = c212;
% c_sym(2,2,2) = c222;

for k = 1:length(q)
    for j = 1:length(q)
        for i = 1:length(q)
            c_sym(i,j,k) = (1/2)*(diff(Dq(k,j), q(i)) + diff(Dq(k,i), q(j)) - diff(Dq(i,j), q(k)));
        end
    end
end

c_sym = simplify(c_sym);

% Coriolis Matrix
disp('Calculating Coriolis Matrix')
% Cqqd = [c_sym(1,1,1)*qd1 + c_sym(1,2,1)*qd2, ...
%         c_sym(2,1,1)*qd1 + c_sym(2,2,1)*qd2; ...
%         c_sym(1,1,2)*qd1 + c_sym(1,2,2)*qd2, ...
%         c_sym(2,1,2)*qd1 + c_sym(2,2,2)*qd2];
    
        
for i = 1:length(q)
    for k = 1:length(q)
        c_sum = c_sym(k,1,i)*qd(1);
        
        for j = 2:length(q)
            c_sum = c_sum + c_sym(k,j,i)*qd(j);
        end
        
        Cqqd(i,k) = c_sum;
    end
end

% Potential Energy
disp('Calculating Potential Energy')
P1 = m1*g*lc1*sin(q1);
P2 = m2*g*(l1*sin(q1) + lc2*sin(q1+q2));
P = P1 + P2;

for i = 1:size(q)
    phi(i,1) = diff(P,q(i));
end

phi = simplify(phi);

% Friction Losses
disp('Calculating Friction Losses')
K1 = fric*[1,0];
K2 = fric*[0,1];

Kqd = [K1; K2];

%% Differential Equations
disp('Calculating Differential Equations')
%tau1 = Dq(1,:)*qdd + Cqqd(1,:)*qd + phi(1);
%tau2 = Dq(2,:)*qdd + Cqqd(2,:)*qd + phi(2);

tau = Dq*qdd + (Cqqd + Kqd)*qd + phi;

sub_vals = [l1, 0.25; ...
            l2, 0.25; ...
            lc1, 0.25/2; ...
            lc2, 0.25/2; ...
            g, 9.80665; ...
            m1, 0.2; ...
            m2, 0.2; ...
            Iz1, 0.2*(0.25^2)/12; ...
            Iz2, 0.2*(0.25^2)/12; ...
            fric, 0.05];
        
tau_sub = subs(tau, sub_vals(:,1), sub_vals(:,2));

disp('Calculating Inverse Dynamics')
% ======================================================================= %
% Robot Modeling and Control
% INVERSE DYNAMICS (p.268)
% Thus the inverse dynamics can be viewed as an input transformation which
% transforms the problem from one of choosing torque input commands, which 
% is difficult, to one of choosing acceleration input commands, which is 
% easy.
% ======================================================================= %

syms u1 u2

toruqe = [u1; u2];

inv_dyn = -(inv(Dq)*(Cqqd*qd + Kqd*qd + phi)) + inv(Dq)*toruqe; % => gives qdd


% Convert to state space
disp('Calculating the State Space System')

%X
A = [0 1 0 0;
     0 0 0 0;
     0 0 0 1;
     0 0 0 0];

B = [0, 0;
     1, 0;
     0, 0;
     0, 1];

%C = [1, 0, 0, 0];
%sys = ss(A, B, C, 0);
%minreal(sys)
%pole(sys)
%eig(A)

Q = [1, 0, 0, 0;
     0, 3, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 3];
 
R = [1, 0;
     0, 1];

%N = zeros(4,2);
%stab_mat = [Q, N;
%            N', R];

%eig(stab_mat)
%chol(stab_mat)

if rank(ctrb(A,B)) == length(A)
    disp('System is controlable')
else
    error('System is not controlable')
end

K = lqr(A,B,Q,R);

% Formulate controller
%A = [0 1 0 0;
%    0 -d/M -m*g/M 0;
%    0 0 0 1;
%    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
%  
%B = [0; 1/M; 0; s*1/(M*L)];
%eig(A)
% 
%Q = [1, 0, 0, 0;
%     0, 10, 0, 0;
%     0, 0, 1, 0;
%     0, 0, 0, 10];
%R = .0001;
% 
%det(ctrb(A,B))
% 
%K = lqr(A,B,Q,R);


%% Run Simulation
disp('Running Simulation')

% Simulation time
tspan = [time_start, time_end];

% Initial state
% [ x; y; z; dx; dy; dz; ...
%   phi; theta; psi; dphi; dtheta; dpsi; ...
%   thetal1; thetal2; dthetal1; dthetal2 ]

%[q1, qd1, q2, qd2]
y0 = [ pi/2+0.01; 0; 0; 0];
yf = [ pi/8; 0; pi/4; 0];

%Define States

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%v = -K*yf;
%v = [0;0];

% Organize to match states
state_vars = [q(1); qd(1); q(2); qd(2)];

% Run the model
%xd1 = x2;
%xd2 = -(inv(Dq(1,:))*(Cqqd(1,:)*x2+phi(1))) + inv(Dq(1,:))*u(1);
%xd3 = x4;
%xd4 = -(inv(Dq(2,:))*(Cqqd(2,:)*x4+phi(2))) + inv(Dq(2,:))*u(2);

%Pre-sub some values to save on time in the sim
Dq_sub = subs(Dq, sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(Cqqd, sub_vals(:,1), sub_vals(:,2));
Kqd_sub = subs(Kqd, sub_vals(:,1), sub_vals(:,2));
phi_sub = subs(phi, sub_vals(:,1), sub_vals(:,2));

do_control = 0;

% Control input simulation
[t,yt] = ode45(@(t,y)arm_run(t, y, -K*(y - yf), Dq_sub, Cqqd_sub, Kqd_sub, phi_sub, state_vars, tspan, do_control),tspan,y0);


%% Render

figure(1);

for k=1:length(t)
    %[q1, qd1, q2, qd2]
    arm_draw(yt(k,:));
    %pause(time_dt)
end







