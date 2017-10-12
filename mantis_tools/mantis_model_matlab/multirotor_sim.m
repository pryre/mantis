%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
time_start = 0;
time_end = 10;
%time_dt = 0.01;

%g = -9.80665; %m/s

% Load system parameters
params = mantis_params('params.yaml');






%% Thoughts

% Define roll, pitch, yaw, and acceleration in the body frame

% Control for acceleration in the world frame







%% Generate the system model
% Model is for a locked frame multirotor, such that only it can only pitch,
% but cannot move in any other rotation or translation
syms m1 g Ix1 Iy1 Iz1 'real'
syms pos posd posdd phi phid phidd theta thetad thetadd psi psid psidd  'real'

q = [pos; phi; theta; psi];
qd = [posd; phi; thetad; psid];
qdd = [posdd; phidd; thetadd; psidd];

% Formulate D(q)
% Translational Kinetic Energy
disp('Calculating Translational Kinetic Energy')
Jvc1 = [1/(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)), 0, 0, 0; ...
        1/(cos(phi)*sin(theta)*sin(psi) - cos(psi)*sin(phi)), 0, 0, 0; ...
        1/(cos(theta)*cos(phi)), 0, 0, 0];

tke1 = m1*((Jvc1')*Jvc1);

% Rotational Kinetic Energy
disp('Calculating Rotational Kinetic Energy')
rkex1 = Ix1*[0, 0, 0, 0; ...
             0, 1, 0, 0; ...
             0, 0, 0, 0; ...
             0, 0, 0, 0];
         
rkey1 = Iy1*[0, 0, 0, 0; ...
             0, 0, 0, 0; ...
             0, 0, 1, 0; ...
             0, 0, 0, 0];
         
rkez1 = Iz1*[0, 0, 0, 0; ...
             0, 0, 0, 0; ...
             0, 0, 0, 0; ...
             0, 0, 0, 1];

% Total Inertia Matrix
disp('Calculating Inertia Matrix')
tke = tke1;
rke = rkex1 + rkey1 + rkez1;
Dq = tke + rke;

% Cristoffel Symbols
disp('Calculating Cristoffel Symbols')
%cijk = (1/2)*(diff(Dq(k,j), qi) + diff(Dq(k,i), qj) - diff(Dq(i,j), qk));
%cijk = cjik
for k = 1:length(q)
    for j = 1:length(q)
        for i = 1:length(q)
            c_sym(i,j,k) = (1/2)*(diff(Dq(k,j), q(i)) + diff(Dq(k,i), q(j)) - diff(Dq(i,j), q(k)));
        end
    end
end

c_sym = simplify(c_sym);

% Coriolis Matrix
disp('Building Coriolis Matrix')

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
P1 = m1*g/(cos(theta)*cos(phi)); %potential of link 1

P = P1;

for i = 1:size(q)
    phi(i,1) = diff(P,q(i));
end

phi = simplify(phi);

% Friction Losses
% disp('Calculating Friction Losses')
% K1 = fric*[1,0];
% K2 = fric*[0,1];
% 
% Kqd = [K1; K2];

%% Differential Equations
disp('Calculating Differential Equations')
%tau1 = Dq(1,:)*qdd + Cqqd(1,:)*qd + phi(1);
%tau2 = Dq(2,:)*qdd + Cqqd(2,:)*qd + phi(2);

tau = Dq*qdd + Cqqd*qd + phi;

sub_vals = [m1, 1.0; ...
            g, 9.80665; ...
            Ix1, (1/12)*1.0*(3*0.1^2 + 0.05^2); ...
            Iy1, (1/12)*1.0*(3*0.1^2 + 0.05^2); ...
            Iz1, 0.5*1.0*0.1^2];
        
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

syms u1 u2 u3 u4

toruqe = [u1; u2; u3; u4];

inv_dyn = -(inv(Dq)*(Cqqd*qd + phi)) + inv(Dq)*toruqe; % => gives qdd

% Convert to state space
disp('Calculating the State Space System')

%X = [x; xd; y; yd; z; zd];
A = [0, 1, 0, 0, 0, 0; ... %xd
     0, 0, 0, 0, 0, 0; ... %xdd
     0, 0, 0, 1, 0, 0; ... %yd
     0, 0, 0, 0, 0, 0; ... %ydd
     0, 0, 0, 0, 0, 1; ... %zd
     0, 0, 0, 0, 0, 0];    %zdd
     
%U = [u1; u2; u3];
B = [0, 0, 0; ... %xd
     1, 0, 0; ... %xdd
     0, 0, 0; ... %yd
     0, 1, 0; ... %ydd
     0, 0, 0; ... %zd
     0, 0, 1];    %zdd

%C = [1, 0, 0, 0];
%sys = ss(A, B, C, 0);
%minreal(sys)
%pole(sys)
%eig(A)
 
Q = [1, 0, 0, 0, 0, 0; ... %x
     0, 5, 0, 0, 0, 0; ... %xd
     0, 0, 1, 0, 0, 0; ... %y
     0, 0, 0, 5, 0, 0; ... %yd
     0, 0, 0, 0, 1, 0; ... %z
     0, 0, 0, 0, 0, 5];    %zd
 
R = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];

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

% [ x; y; z; dx; dy; dz; ...
%   phi; theta; psi; dphi; dtheta; dpsi; ...
%   thetal1; thetal2; dthetal1; dthetal2 ]

%Define States
%[q1, qd1, q2, qd2, q3, qd3, q4, qd4, q5, qd5, q6, qd6]
% Initial state
y0 = [ 0; 0; 0; 0; 0; 0];
% Final state
yf = [ 1; 0; 0; 0; 1; 0];

% Organize to match states
syms px pxd py pyd pz pzd
state_vars = [px; pxd; py; pyd; pz; pzd];

% Run the model
%xd1 = x2;
%xd2 = -(inv(Dq(1,:))*(Cqqd(1,:)*x2+phi(1))) + inv(Dq(1,:))*u(1);
%xd3 = x4;
%xd4 = -(inv(Dq(2,:))*(Cqqd(2,:)*x4+phi(2))) + inv(Dq(2,:))*u(2);

%Pre-sub some values to save on time in the sim
Dq_sub = subs(Dq, sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(Cqqd, sub_vals(:,1), sub_vals(:,2));
phi_sub = subs(phi, sub_vals(:,1), sub_vals(:,2));
[t,y] = ode45(@(t,y)multirotor_run(t, y, -K*(y - yf), Dq_sub, Cqqd_sub, phi_sub, state_vars, tspan),tspan,y0);


%% Render

figure(1);

for k=1:length(t)
    %[q1, qd1, q2, qd2]
    multirotor_draw(y(k,:));
    %pause(time_dt)
end







