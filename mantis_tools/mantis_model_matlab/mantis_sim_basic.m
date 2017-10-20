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
%params = mantis_params('params.yaml');


%% Generate the system model

% Forward Kinematic Model
%Parameters
syms la km kt l1 l2 l3 'real'
%Base link states
syms x y z xd yd zd xdd ydd zdd 'real'
%Arm links
syms r1 r2 r3 r1d r2d r3d r1dd r2dd r3dd 'real'

% Base Link
syms phi theta psi phid thetad psid phidd thetadd psidd 'real'

Rphi = [1, 0, 0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];
Rtheta = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
Rpsi = [cos(phi), -sin(phi), 0;
        sin(phi), cos(phi), 0;
        0, 0, 1];

R0 = Rphi + Rtheta + Rpsi;
p0 = [x; y; z];
g0 = [R0, p0; ...
      zeros(1,3), 1];
g0inv = [R0', -R0'*p0; ...
         zeros(1,3), 1];

     
R0dot = [0, -psid, thetad;
         psid, 0, -phi;
         -thetad, phi, 0];
p0dot = [xd; yd; zd];
g0dot = [R0dot, p0dot; ...
         zeros(1,3), 1];

Vhat = g0inv*g0dot;
     
% Body frame velocity
%syms vbx vby vbz w1 w2 w3 w1d w2d w3d 'real'
%V = [w1; w2; w3; vbx; vby; vbz];
%v_ = [V; r1d; r2d; r3d];

%w_hat = [0, -V(3), V(2); ...
%         V(3), 0, -V(1); ...
%         -V(2), V(1), 0];

%Vhat = [w_hat, [V(4); V(5); V(6)]; ... % Velocity in ineratial frame
%        zeros(1,3), 0];
    
q = [phi; theta; phi; x; y; z; r1; r2; r3];
qd = [phid; thetad; phid; xd; yd; zd; r1d; r2d; r3d];
qdd = [phidd; thetadd; phidd; xdd; ydd; zdd; r1dd; r2dd; r3dd];
     
    
%%
% Mount Link (mounted directly downwards, -pi/2 pitch)
R01 = [cos(r1), 0, sin(r1); ...
       0, 1, 0; ...
       -sin(r1), 0, cos(r1)];
p01 = [l1; 0; 0];
g01 = [R01, p01; ...
       zeros(1,3), 1];
g01inv = [R01', -R01'*p01; ...
         zeros(1,3), 1];

% Arm link A
R12 = [cos(r2), 0, sin(r2); ...
       0, 1, 0; ...
       -sin(r2), 0, cos(r2)];
p12 = [l2; 0; 0];
g12 = [R12, p12; ...
       zeros(1,3), 1];
g12inv = [R12', -R12'*p12; ...
          zeros(1,3), 1];

% Arm link B
R23 = [cos(r3), 0, sin(r3); ...
       0, 1, 0; ...
       -sin(r3), 0, cos(r3)];
p23 = [l3; 0; 0];
g23 = [R23, p23; ...
       zeros(1,3), 1];
   

% Full kinematic solutions
g1 = g0*g01;
g1p = g1(1:3,4);
g1R = g1(1:3,1:3);
g1inv = [g1R', -g1R'*g1p; ...
         zeros(1,3), 1];
g1phat = [0, -g1p(1), g1p(2); ...
          g1p(3), 0, -g1p(1); ...
          -g1p(2), g1p(1), 0];
A1 = [g1R', -g1R'*g1phat; ...
      zeros(3), g1R'];
%Maybe supposed to be diffg1, then multi
J1_1 = diff(g1,r1)*g1inv;
J1_2 = diff(g1,r2)*g1inv;
J1_3 = diff(g1,r3)*g1inv;
J1_sum = J1_1 + J1_2 + J1_3;
J(:,1) = [J1_sum(3,2); J1_sum(1,3); J1_sum(2,1); 0; 0; 0];
  
g2 = g0*g01*g12;
g2p = g2(1:3,4);
g2R = g2(1:3,1:3);
g2inv = [g2R', -g2R'*g2p; ...
         zeros(1,3), 1];
g2phat = [0, -g2p(1), g2p(2); ...
          g2p(3), 0, -g2p(1); ...
          -g2p(2), g2p(1), 0];
A2 = [g2R', -g2R'*g2phat; ...
      zeros(3), g2R'];
%Maybe supposed to be diffg1, then multi
J2_1 = diff(g2,r1)*g2inv;
J2_2 = diff(g2,r2)*g2inv;
J2_3 = diff(g2,r3)*g2inv;
J2_sum = J2_1 + J2_2 + J2_3;
J(:,2) = [J2_sum(3,2); J2_sum(1,3); J2_sum(2,1); 0; 0; 0];
     
g3 = g0*g01*g12*g23;
g3p = g3(1:3,4);
g3R = g3(1:3,1:3);
g3inv = [g3R', -g3R'*g3p; ...
         zeros(1,3), 1];
g3phat = [0, -g3p(1), g3p(2); ...
          g3p(3), 0, -g3p(1); ...
          -g3p(2), g3p(1), 0];
A3 = [g3R', -g3R'*g3phat; ...
      zeros(3), g3R'];
%Maybe supposed to be diffg1, then multi
J3_1 = diff(g3,r1)*g3inv;
J3_2 = diff(g3,r2)*g3inv;
J3_3 = diff(g3,r3)*g3inv;
J3_sum = J3_1 + J3_2 + J3_3;
J(:,3) = [J3_sum(3,2); J3_sum(1,3); J3_sum(2,1); 0; 0; 0];



%% Equations of Motion
% Define inertials
syms m0 m1 m2 m3 'real'

% Base link
IJ0 = [1, 0, 0; ... %Rotational Inertial Tensor
       0, 1, 0; ...
       0, 0, 1];
IT0 = m0*diag(ones(3,1)); %Translational Inertial
I0 = [IJ0, zeros(3); ... %Pose Inertial Tensor
      zeros(3), IT0];

IJ1 = [1, 0, 0; ... %Rotational Inertial Tensor
      0, 1, 0; ...
      0, 0, 1];
IT1 = m1*diag(ones(3,1)); %Translational Inertial
I1 = [IJ1, zeros(3); ... %Pose Inertial Tensor
      zeros(3), IT1];

IJ2 = [1, 0, 0; ... %Rotational Inertial Tensor
      0, 1, 0; ...
      0, 0, 1];
IT2 = m2*diag(ones(3,1)); %Translational Inertial
I2 = [IJ2, zeros(3); ... %Pose Inertial Tensor
      zeros(3), IT2];

IJ3 = [1, 0, 0; ... %Rotational Inertial Tensor
      0, 1, 0; ...
      0, 0, 1];
IT3 = m3*diag(ones(3,1)); %Translational Inertial
I3 = [IJ3, zeros(3); ... %Pose Inertial Tensor
      zeros(3), IT3];

% Formulate M(r)

disp('Calculating Inertia Matrix')

% M_A_A is I0 plus sum of all other links
M_A_A = I0 + A1'*I1*A1 + A2'*I2*A2 + A3'*I3*A3;

M_A_J = A1'*I1*J + A2'*I2*J + A3'*I3*J;

M_J_A = J'*I1*A1 + J'*I2*A2 + J'*I3*A3;

M_J_J = J'*I1*J + J'*I2*J+ J'*I3*J;

%Same as M(r)
Dq = [M_A_A, M_A_J; ...
      M_J_A, M_J_J];


%%

% Cristoffel Symbols
disp('Calculating Cristoffel Symbols')

disp('...0%')
for k = 1:length(q)
    for j = 1:length(q)
        for i = 1:length(q)
            c_sym(i,j,k) = (1/2)*(diff(Dq(k,j), q(i)) + diff(Dq(k,i), q(j)) - diff(Dq(i,j), q(k)));
        end
    end
    
    disp(['... ', num2str(100*k/length(q)), '%'])
end

%%

disp('Simplifying Cristoffel Symbols')

c_sym_simple = sym('t', size(c_sym));
simplify_steps_len = numel(c_sym);

fprintf('Progress:\n');
fprintf(['\n' repmat('.',1,simplify_steps_len) '\n\n']);

parfor i = 1:simplify_steps_len
    c_sym_simple(i) = simplify(c_sym(i));
    fprintf('\b|\n');
    %disp(['... ', num2str(100*i/simplify_steps_len), '%'])
end

%%

% Coriolis Matrix
disp('Calculating Coriolis Matrix')

disp('...0%')
for i = 1:length(q)
    for k = 1:length(q)
        c_sum = c_sym_simple(k,1,i)*qd(1);
        
        for j = 2:length(q)
            c_sum = c_sum + c_sym_simple(k,j,i)*qd(j);
        end
        
        Cqqd(i,k) = c_sum;
    end
    
    disp(['... ', num2str(100*i/length(q)), '%'])
end

save('mantis_sim_basic.mat');

error('break!');

%%

% Potential Energy
disp('Calculating Potential Energy')
P1 = 0;
P2 = m2*g*(l1*sin(qm) + lc2*sin(qm+q2));
P3 = m3*g*(l1*sin(qm) + l2*sin(qm+q2) + lc3*sin(qm+q2+q3));
P = P1 + P2 + P3;

phi1 = diff(P,q1);
phi2 = diff(P,q2);
phi3 = diff(P,q3);

phi = simplify([phi1; phi2; phi3]);


%% Differential Equations
disp('Calculating Differential Equations')
%tau1 = Dq(1,:)*qdd + Cqqd(1,:)*qd + phi(1);
%tau2 = Dq(2,:)*qdd + Cqqd(2,:)*qd + phi(2);
%tau3 = Dq(3,:)*qdd + Cqqd(3,:)*qd + phi(3);

tau = Dq*qdd + Cqqd*qd + phi;

sub_vals = [l1, 0.1; ...
            l2, 0.25; ...
            l3, 0.25; ...
            lc2, 0.25/2; ...
            lc3, 0.25/2; ...
            g, 9.80665; ...
            m1, 2.0; ...
            m2, 0.2; ...
            m3, 0.2; ...
            Iz1, 2.0*(0.25^2)/12; ...
            Iz2, 0.2*(0.25^2)/12; ...
            Iz3, 0.2*(0.25^2)/12];
        
tau_sub = subs(tau, sub_vals(:,1), sub_vals(:,2));

% Formulate controller
%A = [0 1 0 0;
%    0 -d/M -m*g/M 0;
%    0 0 0 1;
%    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
%  
%B = [0; 1/M; 0; s*1/(M*L)];
%eig(A)
% 
%Q = [1 0 0 0;
%    0 1 0 0;
%    0 0 10 0;
%    0 0 0 100];
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

% [q1, qd1, q2, qd2, q3, qd3]
y0 = [ -pi/4; 0; 0; 0; pi/4; 0];

%Define States

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
% [qdd1, qdd2, qdd3]
%u = [0.1; 0.1; 0.1];
u = zeros(3,1);

% Organize to match states
state_vars = [q(1); qd(1); q(2); qd(2); q(3); qd(3); qdd(1); qdd(2); qdd(3);];

% Run the model
%xd1 = x2;
%xd2 = -(inv(Dq(1,:))*(Cqqd(1,:)*x2+phi(1))) + inv(Dq(1,:))*u(1);
%xd3 = x4;
%xd4 = -(inv(Dq(2,:))*(Cqqd(2,:)*x4+phi(2))) + inv(Dq(2,:))*u(2);
%xd5 = x6;
%xd6 = -(inv(Dq(3,:))*(Cqqd(3,:)*x6+phi(3))) + inv(Dq(3,:))*u(3);
Dq_sub = subs(Dq, sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(Cqqd, sub_vals(:,1), sub_vals(:,2));
phi_sub = subs(phi, sub_vals(:,1), sub_vals(:,2));
[t,y] = ode45(@(t,y)mantis_run_pitch_only(t, y, u, Dq_sub, Cqqd_sub, phi_sub, state_vars, tspan),tspan,y0);


%% Render

figure(1);

for k=1:length(t)
% [ x; y; z; dx; dy; dz; ...
%   phi; theta; psi; dphi; dtheta; dpsi; ...
%   thetal1; thetal2; dthetal1; dthetal2 ]
    viz = [0;0;1;0;0;0; ...
           0;-y(k,1);0;0;-y(k,2);0; ...
           y(k,3);y(k,5);y(k,4);y(k,6)];
    mantis_draw(viz, params);
    %pause(time_dt)
end







