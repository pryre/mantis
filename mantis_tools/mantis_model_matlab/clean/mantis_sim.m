%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
% ts = 0;
% te = 0.1;
% dt = 0.005;
ts = 0;
te = 0.005;
dt = 0.005;

%g = -9.80665; %m/s

% Load system parameters
params = mantis_params('params_x4.yaml');
%params = mantis_params('params_x6.yaml');


%% Generate the system model

disp('Generating Parameters')

% Forward Kinematic Model
%Parameters
syms g la km kt l1 l2 m0 m1 m2 'positive'
%Arm links
syms phi bw1 bw1d theta bw2 bw2d psi bw3 bw3d 'real'
syms bx by bz bvx bvxd bvy bvyd bvz bvzd 'real'
syms r1 r2 r1d r2d r1dd r2dd 'real'
%%
% Base Link
g0 = sym('g0_', 4, 'real');
g0inv = [g0(1:3,1:3)', -g0(1:3,1:3)'*g0(1:3,4); ...
         zeros(1,3), 1];
%g0d = sym('g0d', 4, 'real');
%g0dd = sym('g0dd', 4, 'real');

%g0(4,1:3) = 0;
%g0(4,4) = 1;
%g0d(4,1:3) = 0;
%g0dd(4,1:3) = 0;

% Mount Link (mounted directly downwards, pi/2 pitch)
%Rphi = [1, 0, 0;
%        0, cos(phi), -sin(phi);
%        0, sin(phi), cos(phi)];
%Rtheta = [cos(theta), 0, sin(theta);
%          0, 1, 0;
%          -sin(theta), 0, cos(theta)];
%Rpsi = [cos(psi), -sin(psi), 0;
%        sin(psi), cos(psi), 0;
%        0, 0, 1];

%R0 = Rphi*Rtheta*Rpsi;

%g0(1:3,1:3) = R0;
%g0(1:3,4) = [bx;by;bz];

%p0 = [x; y; z];
%g0 = [R0, p0; ...
%      zeros(1,3), 1];
%g0inv = [R0', -R0'*p0; ...
%         zeros(1,3), 1];

     
%R0dot = [0, -psid, thetad;
%         psid, 0, -phid;
%         -thetad, phid, 0];
%p0dot = [xd; yd; zd];
%g0dot = [R0dot, p0dot; ...
%         zeros(1,3), 1];

%Vhat = g0inv*g0dot;
     
% Body frame velocity
%syms vbx vby vbz w1 w2 w3 w1d w2d w3d 'real'
%V = [w1; w2; w3; vbx; vby; vbz];
%v_ = [V; r1d; r2d; r3d];

%w_hat = [0, -V(3), V(2); ...
%         V(3), 0, -V(1); ...
%         -V(2), V(1), 0];

%Vhat = [w_hat, [V(4); V(5); V(6)]; ... % Velocity in ineratial frame
%        zeros(1,3), 0];

q = [phi; theta; psi; bx; by; bz; r1; r2];
qd = [bw1; bw2; bw3; bvx; bvy; bvz; r1d; r2d];
qdd = [bw1d; bw2d; bw3d; bvxd; bvyd; bvzd; r1dd; r2dd];
     
    
%%
disp('Calculating Kinematics')

% ewr1 = [cos(r1), 0, sin(r1); ...
%        0, 1, 0; ...
%        -sin(r1), 0, cos(r1)];
% ewr2 = [cos(r2), 0, sin(r2); ...
%        0, 1, 0; ...
%        -sin(r2), 0, cos(r2)];
% 
% g01_0 = [eye(3), [0;0;0]; ...
%          zeros(1,3), 1];
% g02_0 = [eye(3), [0;0;-l1]; ...
%          zeros(1,3), 1];
% g0e_0 = [eye(3), [0;0;-l1-l2]; ...
%          zeros(1,3), 1];
%    
% % w1 = [0;1;0];
% % w2 = [0;1;0];
% % 
% % q1 = [0;0;0];
% % q2 = [0;0;-l1];
% % 
% % eta1 = [cross(-w1,q1);w1];
% % eta2 = [cross(-w2,q2);w2];
% etr1 = [ewr1, zeros(3,1); ...
%        zeros(1,3), 1];
% etr2 = [ewr2, zeros(3,1); ...
%        zeros(1,3), 1];
% 
% g01 = etr1*g01_0;
% g02 = etr1*etr2*g02_0;
% g0e = etr1*etr2*g0e_0; %TODO Understand this process more!!


g01 = [ cos(r1), 0, sin(r1),  l1*cos(r1); ...
              0, 1,       0,           0; ...
       -sin(r1), 0, cos(r1), -l1*sin(r1); ...
              0, 0,       0,           1];
g12 = [ cos(r2), 0, sin(r2),  l2*cos(r2); ...
              0, 1,       0,           0; ...
       -sin(r2), 0, cos(r2), -l2*sin(r2); ...
              0, 0,       0,           1];
          
%Geometries to the center of each link
g1 = [ cos(r1), 0, sin(r1),  l1*cos(r1)/2; ...
             0, 1,       0,             0; ...
      -sin(r1), 0, cos(r1), -l1*sin(r1)/2; ...
             0, 0,       0,             1];

g2 = g01*[ cos(r2), 0, sin(r2),  l2*cos(r2)/2; ...
                 0, 1,       0,             0; ...
          -sin(r2), 0, cos(r2), -l2*sin(r2)/2; ...
                 0, 0,       0,             1];
             
ge = g01*g12; %End Effector

%Simplify geometries
g1 = simplify(g1);
g2 = simplify(g2);
ge = simplify(ge);

%Calculate inverse geometries
g1p = g1(1:3,4);
g1R = g1(1:3,1:3);
g1inv = [      g1R', -g1R'*g1p; ...
         zeros(1,3),         1];
     
g2p = g2(1:3,4);
g2R = g2(1:3,1:3);
g2inv = [      g2R', -g2R'*g2p; ...
         zeros(1,3),         1];
     
%Simplify inverse geometries
g1inv = simplify(g1inv);
g2inv = simplify(g2inv);
     
%Inverse Adjoint for each link
A1 = [g1R', -g1R'*vee_up(g1p); ...
      zeros(3), g1R'];

A2 = [    g2R', -g2R'*vee_up(g2p); ...
      zeros(3),              g2R'];

%Simplify Adjoints
A1 = simplify(A1);
A2 = simplify(A2);

%Maybe supposed to be diffgX, then multiply
% J1_1 = diff(g01,r1)*g01inv;
% J1_2 = diff(g01,r2)*g01inv;
J1_1 = g1inv*diff(g1,r1);
J1_2 = g1inv*diff(g1,r2);
J1_sum = J1_1 + J1_2;
J(:,1) = [vee_down(J1_sum(1:3,1:3)); 0; 0; 0];

% J2_1 = diff(g02,r1)*g02inv;
% J2_2 = diff(g02,r2)*g02inv;
J2_1 = g2inv*diff(g2,r1);
J2_2 = g2inv*diff(g2,r2);
J2_sum = J2_1 + J2_2;
J(:,2) = [vee_down(J2_sum(1:3,1:3)); 0; 0; 0];

%Simplify Geometric Jacobian
J = simplify(J);


%% Equations of Motion

disp('Defining Inertials')
% Define inertials

syms IT0x IT0y IT0z 'positive'
syms IJ1x IJ1y IJ1z 'positive'
syms IJ2x IJ2y IJ2z 'positive'

% Base link
IJ0 = [IT0x, 0, 0; ... %Rotational Inertial Tensor
       0, IT0y, 0; ...
       0, 0, IT0z];
I0 = [IJ0, zeros(3); ... %Pose Inertial Tensor
      zeros(3), m0*eye(3)];

IJ1 = [IJ1x, 0, 0; ... %Rotational Inertial Tensor
      0, IJ1y, 0; ...
      0, 0, IJ1z];
I1 = [IJ1, zeros(3); ... %Pose Inertial Tensor
      zeros(3), m1*eye(3)];

IJ2 = [IJ2x, 0, 0; ... %Rotational Inertial Tensor
      0, IJ2y, 0; ...
      0, 0, IJ2z];
I2 = [IJ2, zeros(3); ... %Pose Inertial Tensor
      zeros(3), m2*eye(3)];

% Formulate M(r)

disp('Calculating Inertia Matrix')

%disp('Simplify step 1')
% M_A_A is I0 plus sum of all other links

I1s = A1'*I1*A1;
I2s = A2'*I2*A2;

% disp('    Simplifing I1s')
% fprintf('    Progress:\n');
% fprintf(['    ' repmat('.',1,numel(I1s)) '\n    \n']);
% parfor i = 1:numel(I1s)
%     I1s(i) = simplify(I1s(i));
%     fprintf('\b|\n');
% end
% 
% disp('    Simplifing I2s')
% fprintf('    Progress:\n');
% fprintf(['    ' repmat('.',1,numel(I2s)) '\n    \n']);
% parfor i = 1:numel(I2s)
%     I2s(i) = simplify(I2s(i));
%     fprintf('\b|\n');
% end

M_A_A = I0 + I1s + I2s;
M_A_J = A1'*I1*J + A2'*I2*J;
M_J_A = J'*I1*A1 + J'*I2*A2;
M_J_J = J'*I1*J + J'*I2*J;

% disp('Simplify step 2')
% M_A_J = simplify(A1'*I1*J) + simplify(A2'*I2*J);
% 
% disp('Simplify step 3')
% M_J_A = simplify(J'*I1*A1) + simplify(J'*I2*A2);
% 
% disp('Simplify step 4')
% M_J_J = simplify(J'*I1*J) + simplify(J'*I2*J);

%Same as M(r)
Dq = [M_A_A, M_A_J; ...
      M_J_A, M_J_J];
  
disp('    Simplifing Dq')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(Dq)) '\n    \n']);
parfor i = 1:numel(Dq)
    Dq(i) = simplify(Dq(i));
    fprintf('\b|\n');
end

  
%% Cristoffel Symbols

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

% P = zeros(size(q));
% P = sym(P);
% 
% ggrav = sym(diag(ones(4,1)));
% ggrav(3,4) = g;
% ggrav(4,4) = 1;
% f_grav = g0*ggrav;
% P(4) = bx*m0*f_grav(1,4); %bx
% P(5) = by*m0*f_grav(2,4); %by
% P(6) = bz*m0*f_grav(3,4); %bz
% 
% 
% gcg = sym(diag(ones(4,1)));
% 
% gcg(1,4) = l1/2;
% center_height = g0*g01*gcg;
% P(7) = m1*g*center_height(3,4); %r1
% 
% gcg(1,4) = l2/2;
% center_height = g0*g01*g12*gcg;
% P(8) = m2*g*center_height(3,4); %r2
% 
% 
% P_sum = sum(P);
% 
% Nq = zeros(size(q));
% Nq = sym(Nq);
% 
% for i = 1:length(q)
%     Nq(i) = simplify(diff(P_sum,q(i)));
% end
% 
% disp(Nq)


% Gravity can be simulated as an acceleration on the base link
%Nq = sym(zeros(size(q)));

world_grav = [0;0;g];
body_rot = g0(1:3,1:3)';
body_grav = body_rot*world_grav;

accel_grav = [0;0;0;body_grav;0;0];
Nq = Dq*accel_grav;

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

Lqd = [K1; K2; K3; K4; K5; K6; K7; K8];


%% Differential Equations
disp('Calculating Equations of Motion')
%tau1 = Dq(1,:)*qdd + Cqqd(1,:)*qd + phi(1);
%tau2 = Dq(2,:)*qdd + Cqqd(2,:)*qd + phi(2);
%tau3 = Dq(3,:)*qdd + Cqqd(3,:)*qd + phi(3);

tauq = Dq*qdd + (Cqqd + Lqd)*qd + Nq;


%% Formulate controller

% M is a motor map, (num_motors + num_servos)x(6 + num_servos)
mr = cos(pi/4);
alen = params.arm.length;
motor_map = [-mr*alen*kt, -mr*alen*kt,  km, 0, 0, kt; ... % TODO: CHANGE HERE
              mr*alen*kt,  mr*alen*kt,  km, 0, 0, kt; ...
              mr*alen*kt, -mr*alen*kt, -km, 0, 0, kt; ...
              mr*alen*kt,  mr*alen*kt, -km, 0, 0, kt];

Mm = [motor_map, zeros(params.motor.num, params.arm.links); ...
      zeros(params.arm.links,6), eye(params.arm.links)];
  
%Build pose solvers
Gq = cell(params.arm.links + 1, 1);
Gq(1) = {g0}; %Base link
Gq(2) = {g0*g01}; %End of link 1
Gq(3) = {g0*g01*g12}; %End of link 2
for i = 1:numel(Gq)
    Gq{i} = simplify(Gq{i});
end
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


%% Prep equations for faster calculations

sub_vals = [km, 0.5; ...
            kt, 1/(params.motor.num*params.motor.max_thrust); ... % TODO: CHANGE HERE AS WELL
            la, params.frame.motor_arm_length; ...
            l1, params.arm.length; ...
            l2, params.arm.length; ...
            m0, 2.0; ...
            m1, 0.005; ...
            m2, 0.2; ...
            g, 9.80665; ...
            IT0x, (0.05^2 + la^2)/12; ...
            IT0y, (0.05^2 + la^2)/12; ...
            IT0z, la^2/2; ...
            IJ1x, (0.05^2 + l1^2)/12; ...
            IJ1y, (0.05^2 + l1^2)/12; ...
            IJ1z, (2*0.05^2)/12; ...
            IJ2x, (0.05^2 + l1^2)/12; ...
            IJ2y, (0.05^2 + l1^2)/12; ...
            IJ2z, (2*0.05^2)/12];
        
tauq_sub = subs(subs(tauq, sub_vals(:,1), sub_vals(:,2)), sub_vals(:,1), sub_vals(:,2));

Dq_sub = subs(subs(Dq, sub_vals(:,1), sub_vals(:,2)), sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(subs(Cqqd, sub_vals(:,1), sub_vals(:,2)), sub_vals(:,1), sub_vals(:,2));
Nq_sub = subs(subs(Nq, sub_vals(:,1), sub_vals(:,2)), sub_vals(:,1), sub_vals(:,2));

Mm_sub = subs(subs(Mm, sub_vals(:,1), sub_vals(:,2)), sub_vals(:,1), sub_vals(:,2));
Gq_sub = cell(size(Gq));
for i = 1:numel(Gq)
    Gq_sub{i} = subs(subs(Gq{i}, sub_vals(:,1), sub_vals(:,2)), sub_vals(:,1), sub_vals(:,2));
end


%% Prepare Matrix solvers

Dq_eq = cell(size(Dq));
Cqqd_eq = cell(size(Cqqd));
N_eq = cell(size(Nq));

G_eq = cell(size(Gq));

state_vars =[r1; r2; qd];
pose_vars = [g0(:);r1;r2];

disp('    Preparing Dq solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(Dq)) '\n    \n']);
parfor i = 1:numel(Dq)
    Dq_eq(i) = {matlabFunction(Dq_sub(i), 'Vars', state_vars)};
    fprintf('\b|\n');
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
fprintf(['    ' repmat('.',1,numel(Nq)) '\n    \n']);
parfor i = 1:numel(Nq)
    N_eq(i) = {matlabFunction(Nq_sub(i), 'Vars', pose_vars)};
    fprintf('\b|\n');
end

disp('    Preparing Pose solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(Gq)) '\n    \n']);
parfor i = 1:numel(Gq)
    G_eq{i} = cell(size(Gq{i}));
    for j = 1:numel(Gq{i})
        G_eq{i}(j) = {matlabFunction(Gq_sub{i}(j), 'Vars', pose_vars)};
    end
    fprintf('\b|\n');
end


%% Generate C Code
function_gen_mat(Dq, 'Dq');
function_gen_mat(Cqqd, 'Cqqd');
function_gen_mat(sym(Lqd), 'Lqd');
function_gen_mat(Nq, 'Nq');

for i = 1:numel(Gq)
    function_gen_mat(Gq{i}, ['G', num2str(i-1)]);
end


%% Run Simulation
disp('Preparing Simulation')

phi0 = 0;
theta0 = 0;
psi0 = 0;
x = 0;
y = 0;
z = 1.0;

Rphi = [1, 0, 0;
        0, cos(phi0), -sin(phi0);
        0, sin(phi0), cos(phi0)];
Rtheta = [cos(theta0), 0, sin(theta0);
          0, 1, 0;
          -sin(theta0), 0, cos(theta0)];
Rpsi = [cos(psi0), -sin(psi0), 0;
        sin(psi0), cos(psi0), 0;
        0, 0, 1];

Ry0 = Rphi*Rtheta*Rpsi;
py0 = [x; y; z];
gy0 = [Ry0, py0; ...
       zeros(1,3), 1];

vy0 = zeros(6,1); % w1, w2, w3, bvx, bvy, bvz
%gdy0 = [0.1;0;0;0;0;0]; % w1, w2, w3, bvx, bvy, bvz

r0 = [pi/2 + 0.1; 0]; %r1, r2

rd0 = [0; 0]; %r1d, r2d

y0 = [gy0(:); r0; vy0; rd0];
         
%Define States

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
% [qdd1, qdd2, qdd3]
%u = [0.1; 0.1; 0.1];

% Input accelerations
ua = zeros(size(q));




%%
disp('Running Simulation')

%[ts,ys] = ode45(@(tfb,yfb)mantis_run(tfb, yfb, u, Dq_eq, Cqqd_eq, Kqd, N_eq, tspan, do_control),tspan,y0);

t = ts:dt:te;
y = zeros(length(y0), length(t));
u = zeros(params.motor.num + params.arm.links, length(t));
y(:,1) = y0;

for k = 1:(length(t)-1)
    gk = reshape(y(1:16,k),[4,4]);
    rk = y(17:18,k);
    vk = y(19:26,k);
    
    D = zeros(size(Dq_eq));
    C = zeros(size(Cqqd_eq));
    N = zeros(size(N_eq));
    
    for i = 1:numel(Dq_eq)
                        %r1, r2, bw1, bw2, bw3, bvx, bvy, bvz, r1d, r2d
        D(i) = Dq_eq{i}(rk(1), rk(2), vk(1), vk(2), vk(3), vk(4), vk(5), vk(6), vk(7), vk(8));
    end

    for i = 1:numel(C)
                          %r1, r2, bw1, bw2, bw3, bvx, bvy, bvz, r1d, r2d
        C(i) = Cqqd_eq{i}(rk(1), rk(2), vk(1), vk(2), vk(3), vk(4), vk(5), vk(6), vk(7), vk(8));
    end
    
    L = Lqd;
   
    for i = 1:numel(N)
        %g, r1, r2
        %g01_1,g02_1,g03_1,g04_1,g01_2,g02_2,g03_2,g04_2,g01_3,g02_3,g03_3,g04_3,g01_4,g02_4,g03_4,g04_4,r1,r2
        N(i) = N_eq{i}(gk(1,1), gk(2,1), gk(3,1), gk(4,1), ...
                       gk(1,2), gk(2,2), gk(3,2), gk(4,2), ...
                       gk(1,3), gk(2,3), gk(3,3), gk(4,3), ...
                       gk(1,4), gk(2,4), gk(3,4), gk(4,4), ...
                       rk(1), rk(2));
    end
    
    M = double(Mm_sub);
    
    % Inverse Dynamics control
    tau = D*ua + (C + L)*vk + N;
    u(:,k) = M*tau;
    
    % Simulate 1 time step
    %y(:,k+1)= mantis_run(dt, y(:,k), D, C, L, N);
    
    y(:,k+1) = y(:,k);
    
    disp([num2str(100*(k/length(t))), '%'])
end
disp('100%')

disp(u)


%% Render

figure(1);

for k=1:length(t)
    gk = reshape(y(1:16,k),[4,4]);
    rk = y(17:18,k);
    fl = params.frame.motor_arm_length;
    
    hold off; % Clean the display area
    plot3([0,0.5], [0,0], [0,0], 'color', 'r', 'linewidth', 1);
    hold on;
    plot3([0,0], [0,0.5], [0,0], 'color', 'g', 'linewidth', 1);
    plot3([0,0], [0,0], [0,0.5], 'color', 'b', 'linewidth', 1); 
    
    %Calculate endpoint poses
    G = cell(size(G_eq));
    
    for i = 1:numel(G)
        %g, r1, r2
        %g01_1,g02_1,g03_1,g04_1,g01_2,g02_2,g03_2,g04_2,g01_3,g02_3,g03_3,g04_3,g01_4,g02_4,g03_4,g04_4,r1,r2
        G{i} = zeros(size(G_eq{i}));
        
        for j = 1:numel(G_eq{i})
            G{i}(j) = G_eq{i}{j}(gk(1,1), gk(2,1), gk(3,1), gk(4,1), ...
                                 gk(1,2), gk(2,2), gk(3,2), gk(4,2), ...
                                 gk(1,3), gk(2,3), gk(3,3), gk(4,3), ...
                                 gk(1,4), gk(2,4), gk(3,4), gk(4,4), ...
                                 rk(1), rk(2));
        end
    end
    
    %Plot frame
    for i = 1:params.motor.num
        ra = params.frame.map(i);
        len_a = params.frame.motor_arm_length;
        
        ga = [cos(ra), -sin(ra), 0, len_a*cos(ra); ...
              sin(ra),  cos(ra), 0, len_a*sin(ra); ...
                    0,        0, 1,             0; ...
                    0,        0, 0,             1];

        pa = G{1}*ga;

        plot3([G{1}(1,4),pa(1,4)], [G{1}(2,4),pa(2,4)], [G{1}(3,4),pa(3,4)], 'color', 'k', 'linewidth', 2);
    end
    
    %Plot arm links
    for i = 1:(params.arm.links)
        plot3([G{i}(1,4),G{i+1}(1,4)], [G{i}(2,4),G{i+1}(2,4)], [G{i}(3,4),G{i+1}(3,4)], 'color', 'b', 'linewidth', 2);
    end
    
    %Draw figure
    ax_s = params.plot.size / 2;
    
    axis([-ax_s, ax_s, -ax_s, ax_s, 0, 2*ax_s]);
    axis('square')
    
    drawnow;
    
    disp([num2str(100*(k/length(t))), '%'])

end







