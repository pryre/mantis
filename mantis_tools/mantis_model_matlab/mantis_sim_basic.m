%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
time_start = 0;
time_end = 2;
%time_dt = 0.01;

%g = -9.80665; %m/s

% Load system parameters
%params = mantis_params('params.yaml');


%% Generate the system model

disp('Generating Parameters')

% Forward Kinematic Model
%Parameters
syms g la km kt l1 l2 m0 m1 m2 'positive'
%Base link states
syms x y z xd yd zd xdd ydd zdd 'real'
%Arm links
syms r1 r2 r1d r2d r1dd r2dd 'real'

% Base Link
syms phi theta psi phid thetad psid phidd thetadd psidd 'real'


% Mount Link (mounted directly downwards, pi/2 pitch)
Rphi = [1, 0, 0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];
Rtheta = [cos(theta + pi/2), 0, sin(theta + pi/2);
          0, 1, 0;
          -sin(theta + pi/2), 0, cos(theta + pi/2)];
Rpsi = [cos(psi), -sin(psi), 0;
        sin(psi), cos(psi), 0;
        0, 0, 1];

R0 = Rphi + Rtheta + Rpsi;
p0 = [x; y; z];
g0 = [R0, p0; ...
      zeros(1,3), 1];
g0inv = [R0', -R0'*p0; ...
         zeros(1,3), 1];

     
R0dot = [0, -psid, thetad;
         psid, 0, -phid;
         -thetad, phid, 0];
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
    
q = [phi; theta; psi; x; y; z; r1; r2];
qd = [phid; thetad; psid; xd; yd; zd; r1d; r2d];
qdd = [phidd; thetadd; psidd; xdd; ydd; zdd; r1dd; r2dd];
     
    
%%
disp('Calculating Kinematics')

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
A1 = simplify(A1);

%Maybe supposed to be diffg1, then multi
J1_1 = diff(g1,r1)*g1inv;
J1_2 = diff(g1,r2)*g1inv;
J1_sum = J1_1 + J1_2;
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
A2 = simplify(A2);

%Maybe supposed to be diffg1, then multi
J2_1 = diff(g2,r1)*g2inv;
J2_2 = diff(g2,r2)*g2inv;
J2_sum = J2_1 + J2_2;
J(:,2) = [J2_sum(3,2); J2_sum(1,3); J2_sum(2,1); 0; 0; 0];


%% Equations of Motion

disp('Defining Inertials')
% Define inertials

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

% Formulate M(r)

disp('Calculating Inertia Matrix')

disp('Simplify step 1')
% M_A_A is I0 plus sum of all other links

I1s = A1'*I1*A1;
I2s = A2'*I2*A2;

disp('    Simplifing I1s')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(I1s)) '\n    \n']);
parfor i = 1:numel(I1s)
    I1s(i) = simplify(I1s(i));
    fprintf('\b|\n');
end

disp('    Simplifing I2s')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(I2s)) '\n    \n']);
parfor i = 1:numel(I2s)
    I2s(i) = simplify(I2s(i));
    fprintf('\b|\n');
end

M_A_A = I0 + I1s + I2s;

disp('Simplify step 2')
M_A_J = simplify(A1'*I1*J) + simplify(A2'*I2*J);

disp('Simplify step 3')
M_J_A = simplify(J'*I1*A1) + simplify(J'*I2*A2);

disp('Simplify step 4')
M_J_J = simplify(J'*I1*J) + simplify(J'*I2*J);

%Same as M(r)
Dq = [M_A_A, M_A_J; ...
      M_J_A, M_J_J];

  %%
  
disp('    Simplifing Dq')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(Dq)) '\n    \n']);
parfor i = 1:numel(Dq)
    Dq(i) = simplify(Dq(i));
    fprintf('\b|\n');
end
  
%%

% Cristoffel Symbols
disp('Calculating Cristoffel Symbols')

disp('... 0%')
for k = 1:length(q)
    for j = 1:length(q)
        for i = 1:length(q)
            c_sym(i,j,k) = (1/2)*(diff(Dq(k,j), q(i)) + diff(Dq(k,i), q(j)) - diff(Dq(i,j), q(k)));
        end
    end
    
    disp(['... ', num2str(100*k/length(q)), '%'])
end

%%

%disp('Simplifying Cristoffel Symbols')

%c_sym_simple = sym('t', size(c_sym));
%simplify_steps_len = numel(c_sym);

%fprintf('Progress:\n');
%fprintf(['\n' repmat('.',1,simplify_steps_len) '\n\n']);

%parfor i = 1:simplify_steps_len
%    c_sym_simple(i) = simplify(c_sym(i));
%    fprintf('\b|\n');
%    %disp(['... ', num2str(100*i/simplify_steps_len), '%'])
%end

%%

% Coriolis Matrix
disp('Calculating Coriolis Matrix')

disp('... 0%')
for i = 1:length(q)
    for k = 1:length(q)
        c_sum = c_sym(k,1,i)*qd(1);
        
        for j = 2:length(q)
            c_sum = c_sum + c_sym(k,j,i)*qd(j);
        end
        
        Cqqd(i,k) = c_sum;
    end
    
    disp(['... ', num2str(100*i/length(q)), '%'])
end

%save('mantis_sim_basic.mat');

%%

% Potential Energy
disp('Calculating Potential Energy')

% Gravity effect on links

P = zeros(size(q));
P = sym(P);

gcg = sym(diag(ones(4,1)));


P(6) = m0*g; %z

gcg(1,4) = l1;
center_height = g0*gcg;
P(7) = m1*g*center_height(3,4); %r1

gcg(1,4) = l2;
center_height = g0*g01*gcg;
P(8) = m1*g*center_height(3,4); %r2

P_sum = sum(P);


N = zeros(size(q));
N = sym(N);

for i = 1:length(q)
    N(i) = simplify(diff(P_sum,q(i)));
end


%% Friction Losses
disp('Calculating Friction Losses')
fric = 0.05;

%Losses on last 2 links only
K1 = zeros(1,8);
K2 = zeros(1,8);
K3 = zeros(1,8);
K4 = zeros(1,8);
K5 = zeros(1,8);
K6 = zeros(1,8);
K7 = fric*[0,0,0,0,0,0,1,0];
K8 = fric*[0,0,0,0,0,0,0,1];

Kqd = [K1; K2; K3; K4; K5; K6; K7; K8];


%% Differential Equations
disp('Calculating Equations of Motion')
%tau1 = Dq(1,:)*qdd + Cqqd(1,:)*qd + phi(1);
%tau2 = Dq(2,:)*qdd + Cqqd(2,:)*qd + phi(2);
%tau3 = Dq(3,:)*qdd + Cqqd(3,:)*qd + phi(3);

tau = Dq*qdd + (Cqqd + Kqd)*qd + N;

sub_vals = [km, 0; ...
            kt, 0; ...
            la, 0.275; ...
            l1, 0.2; ...
            l2, 0.2; ...
            m0, 2.0; ...
            m1, 0.005; ...
            m2, 0.2; ...
            g, 9.80665];
        
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
disp('Preparing Simulation')

% Simulation time
tspan = [time_start, time_end];

% Initial state
% [ phi; phid;
%   theta; thetad;
%   psi; dpsi; 
%   x; xd; 
%   y; yd;
%   z; zd;
%   r1; r1d; 
%   r2; r2d ]

y0 = [ 0; 0;
       0; 0;
       0; 0; 
       0; 0;
       1; 0;
       0; 0;
       0; 0;
       0; 0];

%Define States

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
% [qdd1, qdd2, qdd3]
%u = [0.1; 0.1; 0.1];
u = zeros(size(q));

% Run the model
Dq_sub = subs(Dq, sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(Cqqd, sub_vals(:,1), sub_vals(:,2));
N_sub = subs(N, sub_vals(:,1), sub_vals(:,2));

Dq_eq = cell(size(Dq));
Cqqd_eq = cell(size(Cqqd));
N_eq = cell(size(N));

%TODO: Losses

state_vars =[q'; qd'];
state_vars = state_vars(:);

%%
disp('    Preparing Dq solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(Dq)) '\n    \n']);
parfor i = 1:numel(Dq)
    Dq_eq(i) = {matlabFunction(Dq_sub(i), 'Vars', state_vars)};
    fprintf('\b|\n');
    %disp(['    ... ', num2str(100*i/numel(Dq)), '%'])
end

disp('    Preparing Cqqd solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(Cqqd)) '\n    \n']);
parfor i = 1:numel(Cqqd)
    Cqqd_eq(i) = {matlabFunction(Cqqd_sub(i), 'Vars', state_vars)};
    fprintf('\b|\n');
end

disp('    Preparing N solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(N)) '\n    \n']);
parfor i = 1:numel(N)
    N_eq(i) = {matlabFunction(N_sub(i), 'Vars', state_vars)};
    fprintf('\b|\n');
end

%%

disp('Running Simulation')
    
do_control = 0;
vel_body_rot = 0;

[ts,ys] = ode45(@(tfb,yfb)mantis_run(tfb, yfb, u, Dq_eq, Cqqd_eq, N_eq, tspan, do_control),tspan,y0);


%% Render

figure(1);

for k=1:length(ts)
% [ x; y; z; dx; dy; dz; ...
%   phi; theta; psi; dphi; dtheta; dpsi; ...
%   thetal1; thetal2; dthetal1; dthetal2 ]
    viz = [0;0;1;0;0;0; ...
           0;-ys(k,1);0;0;-ys(k,2);0; ...
           ys(k,3);ys(k,5);ys(k,4);ys(k,6)];
    mantis_draw(viz, params);
    %pause(time_dt)
end







