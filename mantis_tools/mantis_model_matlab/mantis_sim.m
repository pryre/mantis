% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

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
% params = mantis_params('params_x6.yaml');


%% Generate the system model

disp('Generating Parameters')

% Forward Kinematic Model
%Parameters
syms  g l0 l1 l2 lc1 lc2 m0 m1 m2 'positive'
%Arm links
syms bra brb bw1 bw1d brc bw2 bw2d brd bw3 bw3d 'real'
syms bx by bz bvx bvxd bvy bvyd bvz bvzd 'real'
syms r1 r2 r1d r2d r1dd r2dd 'real'
%%
% % Base Link
% gb = sym('gb_', 4, 'real');
% gbinv = inverse_trans(gb);
% gbinv = [gb(1:3,1:3)', -gb(1:3,1:3)'*gb(1:3,4); ...
%          zeros(1,3), 1];
%

%gbd = sym('gbd', 4, 'real');
%gbdd = sym('gbdd', 4, 'real');

%gb(4,1:3) = 0;
%gb(4,4) = 1;
%gbd(4,1:3) = 0;
%gbdd(4,1:3) = 0;

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

%gb(1:3,1:3) = R0;
%gb(1:3,4) = [bx;by;bz];

%p0 = [x; y; z];
%gb = [R0, p0; ...
%      zeros(1,3), 1];
%gbinv = [R0', -R0'*p0; ...
%         zeros(1,3), 1];


%R0dot = [0, -psid, thetad;
%         psid, 0, -phid;
%         -thetad, phid, 0];
%p0dot = [xd; yd; zd];
%gbdot = [R0dot, p0dot; ...
%         zeros(1,3), 1];

%Vhat = gbinv*gbdot;

% Body frame velocity
%syms vbx vby vbz w1 w2 w3 w1d w2d w3d 'real'
%V = [w1; w2; w3; vbx; vby; vbz];
%v_ = [V; r1d; r2d; r3d];

%w_hat = [0, -V(3), V(2); ...
%         V(3), 0, -V(1); ...
%         -V(2), V(1), 0];

%Vhat = [w_hat, [V(4); V(5); V(6)]; ... % Velocity in ineratial frame
%        zeros(1,3), 0];

q = [bx; by; bz; brb; brc; brd; r1; r2];
qd = [bvx; bvy; bvz; bw1; bw2; bw3; r1d; r2d];
qdd = [bvxd; bvyd; bvzd; bw1d; bw2d; bw3d; r1dd; r2dd];

bv = [bvx; bvy; bvz];
bw = [bw1; bw2; bw3];

%%
disp('Calculating Kinematics')

% ewr1 = [cos(r1), 0, sin(r1); ...
%        0, 1, 0; ...
%        -sin(r1), 0, cos(r1)];
% ewr2 = [cos(r2), 0, sin(r2); ...
%        0, 1, 0; ...
%        -sin(r2), 0, cos(r2)];
%
% gb1_0 = [eye(3), [0;0;0]; ...
%          zeros(1,3), 1];
% gb2_0 = [eye(3), [0;0;-l1]; ...
%          zeros(1,3), 1];
% gbe_0 = [eye(3), [0;0;-l1-l2]; ...
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
% gb1 = etr1*gb1_0;
% gb2 = etr1*etr2*gb2_0;
% gbe = etr1*etr2*gbe_0; %TODO Understand this process more!!

%
% gb1 = [ cos(r1), 0, sin(r1),  l1*cos(r1); ...
%               0, 1,       0,           0; ...
%        -sin(r1), 0, cos(r1), -l1*sin(r1); ...
%               0, 0,       0,           1];
% g12 = [ cos(r2), 0, sin(r2),  l2*cos(r2); ...
%               0, 1,       0,           0; ...
%        -sin(r2), 0, cos(r2), -l2*sin(r2); ...
%               0, 0,       0,           1];
%
% %Geometries to the center of each link, relative to the base link
% g1 = [ cos(r1), 0, sin(r1),  l1*cos(r1)/2; ...
%              0, 1,       0,             0; ...
%       -sin(r1), 0, cos(r1), -l1*sin(r1)/2; ...
%              0, 0,       0,             1];
%
% g2 = gb1*[ cos(r2), 0, sin(r2),  l2*cos(r2)/2; ...
%                  0, 1,       0,             0; ...
%           -sin(r2), 0, cos(r2), -l2*sin(r2)/2; ...
%                  0, 0,       0,             1];
%
% ge = gb1*g12; %End Effector

gb = sym('gb_', 4, 'real');
gb(4,:) = [zeros(1,3), 1];
gbinv = inverse_trans(gb);
gb1 = dh_gen(l0,         0,  0, pi/2);

%Joint geometries
g12 = dh_gen( 0, r1 - pi/2, l1,    0);
g23 = dh_gen( 0,        r2, l2,    0);

%Link CoM geometries
g1l1 = dh_gen(0, r1 - pi/2, lc1, 0);
g2l2 = dh_gen(0,        r2, lc2, 0);

%Link base-relative geometries
gbl1 = gb1*g1l1;
gbl2 = gb1*g12*g2l2;

%Simplify geometries
gbl1 = simplify(gbl1);
gbl2 = simplify(gbl2);

%Calculate inverse geometries
% g1p = g1(1:3,4);
% g1R = g1(1:3,1:3);
% g1inv = [      g1R', -g1R'*g1p; ...
%          zeros(1,3),         1];
%
% g2p = g2(1:3,4);
% g2R = g2(1:3,1:3);
% g2inv = [      g2R', -g2R'*g2p; ...
%          zeros(1,3),         1];

gbl1inv = inverse_trans(gbl1);
gbl2inv = inverse_trans(gbl2);

%Simplify inverse geometries
gbl1inv = simplify(gbl1inv);
gbl2inv = simplify(gbl2inv);

%Inverse Adjoint for each link
% A1 = [g1R', -g1R'*vee_up(g1p); ...
%       zeros(3), g1R'];
%
% A2 = [    g2R', -g2R'*vee_up(g2p); ...
%       zeros(3),              g2R'];

A1 = adjoint_trans(gbl1inv);
A2 = adjoint_trans(gbl2inv);

%Simplify Adjoints
A1 = simplify(A1);
A2 = simplify(A2);

%Maybe supposed to be diffgX, then multiply
% J1_1 = diff(g1,r1)*g1inv;
% J1_2 = diff(g1,r2)*g1inv;
% J1_1 = diff(g1inv,r1)*g1;
% J1_2 = diff(g1inv,r2)*g1;
% J1_1 = g1inv*diff(g1,r1);
% J1_2 = g1inv*diff(g1,r2);
% J1(:,1) = [vee_down(J1_1(1:3,1:3)); J1_1(1:3,4)];
% J1(:,2) = [vee_down(J1_2(1:3,1:3)); J1_2(1:3,4)];
% J1_sum = J1_1 + J1_2;
% J(:,1) = [vee_down(J1_sum(1:3,1:3)); 0; 0; 0];

% J2_1 = diff(g2,r1)*g2inv;
% J2_2 = diff(g2,r2)*g2inv;
% J2_1 = diff(g2inv,r1)*g2;
% J2_2 = diff(g2inv,r2)*g2;
% J2_1 = g2inv*diff(g2,r1);
% J2_2 = g2inv*diff(g2,r2);
% J2(:,1) = [vee_down(J2_1(1:3,1:3)); J2_1(1:3,4)];
% J2(:,2) = [vee_down(J2_2(1:3,1:3)); J2_2(1:3,4)];
% J2_sum = J2_1 + J2_2;
% J(:,2) = [vee_down(J2_sum(1:3,1:3)); 0; 0; 0];
J1 = jacobian_gen(gbl1,[r1;r2]);
J2 = jacobian_gen(gbl2,[r1;r2]);

%Simplify Geometric Jacobian
J1 = simplify(J1);
J2 = simplify(J2);

%Joint jacobians
Jj1 = jacobian_gen(gb1,[r1;r2]);
Jj1 = simplify(Jj1);

Jj2 = jacobian_gen(gb1*g12,[r1;r2]);
Jj2 = simplify(Jj2);

%End effector jacobian
Je = jacobian_gen(gb1*g12*g23,[r1;r2]);
Je = simplify(Je);


%% Equations of Motion

disp('Defining Inertials')
% Define inertials

syms IJ0x IJ0y IJ0z 'positive'
syms IJ1x IJ1y IJ1z 'positive'
syms IJ2x IJ2y IJ2z 'positive'

% Base link
% IJ0 = [IJ0x, 0, 0; ... %Rotational Inertial Tensor
%        0, IJ0y, 0; ...
%        0, 0, IJ0z];
% I0 = [IJ0, zeros(3); ... %Pose Inertial Tensor
%       zeros(3), m0*eye(3)];
%
% IJ1 = [IJ1x, 0, 0; ... %Rotational Inertial Tensor
%       0, IJ1y, 0; ...
%       0, 0, IJ1z];
% I1 = [IJ1, zeros(3); ... %Pose Inertial Tensor
%       zeros(3), m1*eye(3)];
%
% IJ2 = [IJ2x, 0, 0; ... %Rotational Inertial Tensor
%       0, IJ2y, 0; ...
%       0, 0, IJ2z];
% I2 = [IJ2, zeros(3); ... %Pose Inertial Tensor
%       zeros(3), m2*eye(3)];

I0 = inertial_gen(m0,IJ0x,IJ0y,IJ0z);
I1 = inertial_gen(m1,IJ1x,IJ1y,IJ1z);
I2 = inertial_gen(m2,IJ2x,IJ2y,IJ2z);

% Formulate M(r)

disp('Calculating Inertia Matrix')

%disp('Simplify step 1')
% M_A_A is I0 plus sum of all other links
%
% I1s = A1'*I1*A1;
% I2s = A2'*I2*A2;
%
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

M_A_A = I0 + A1'*I1*A1 + A2'*I2*A2;
M_A_J = A1'*I1*J1 + A2'*I2*J2;
M_J_A = J1'*I1*A1 + J2'*I2*A2;
M_J_J = J1'*I1*J1 + J2'*I2*J2;

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
disp('Calculating  Matrix')

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

%%
% A matematical introduction to robotic manipulation (pg. 167 [185])
% A matematical introduction to robotic manipulation (pg. 277 [295])

% vee_up(bw)*m0%*bv
% vee_up(bw)*IJ0%*bw
% vee_up(bw)*IJ0%bw
% simplify(cross(bw,(IJ0*bw)))
% simplify(simplify(0.5*(vee_up(bw)*IJ0 - IJ0*vee_up(bw))*bw))
% simplify(vee_up(bw)*(IJ0*bw))
% simplify((vee_up(bw)*IJ0)*bw)
% 
% Cqqd_base =  [m0*vee_up(bw),       zeros(3);
%                    zeros(3), vee_up(bw)*I0(4:6,4:6)]

Cqqd_bm = [vee_up(bw), zeros(3);
            zeros(3), vee_up(bw)];
Cqqd_0 = Cqqd_bm*I0; %Cqqd_base
Cqqd(1:6,1:6) = Cqqd(1:6,1:6) + Cqqd_0;

% simplify(Cqqd_base == Cqqd_bm*I0)
%                (vee_up(bw)*IJ0 - IJ0*vee_up(bw)) 
% simplify(Cqqd_base(4:6,4:6)*bw == vee_up(bw)*IJ0*bw)

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
% f_grav = gb*ggrav;
% P(4) = bx*m0*f_grav(1,4); %bx
% P(5) = by*m0*f_grav(2,4); %by
% P(6) = bz*m0*f_grav(3,4); %bz
%
%
% gcg = sym(diag(ones(4,1)));
%
% gcg(1,4) = l1/2;
% center_height = gb*gb1*gcg;
% P(7) = m1*g*center_height(3,4); %r1
%
% gcg(1,4) = l2/2;
% center_height = gb*gb1*g12*gcg;
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

wgv = [0;0;1];
bgv = gb(1:3,1:3)*wgv;
% theta_g = acos(dot(world_grav,body_grav)/norm(body_grav));
% bgz = g/cos(theta_g);
bgz = g/(dot(wgv,bgv));


accel_grav = [0;0;bgz;0;0;0;0;0];
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
Mm = [params.motor_map, zeros(params.motor.num, params.arm.links); ...
      zeros(params.arm.links,6), eye(params.arm.links)];

%Build pose solvers
Gq = cell(params.arm.links + 1, 1);
Gq(1) = {gb}; %Base link
Gq(2) = {gb*gb1}; %Base link
Gq(3) = {gb*gb1*g12}; %End of link 1
Gq(4) = {gb*gb1*g12*g23}; %End of link 2
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

sub_vals = [l0, params.arm.mount.distance; ...
            l1, params.arm.length; ...
            l2, params.arm.length; ...
            lc1, params.arm.length/2; ...
            lc2, params.arm.length/2; ...
            m0, params.frame.mass; ...
            m1, params.arm.mass; ...
            m2, params.arm.mass; ...
            g, 9.80665; ...
            IJ0x, 0.02961; ... %m0*(params.frame.height^2 + 3*params.frame.motor_radius^2)/12; ...
            IJ0y, 0.02961; ... %m0*(params.frame.height^2 + 3*params.frame.motor_radius^2)/12; ...
            IJ0z, 0.05342; ... %m0*params.frame.motor_radius^2/2; ...
            IJ1x, 0.00001345; ...
            IJ1y, 0.00002333; ...
            IJ1z, 0.00002098; ...
            IJ2x, 0.00001345; ...
            IJ2y, 0.00002333; ...
            IJ2z, 0.00002098];

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
pose_vars = [symvar(gb)';r1;r2];
grav_vars = [gb(3,1:3)';r1;r2];

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
    N_eq(i) = {matlabFunction(Nq_sub(i), 'Vars', grav_vars)};
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
disp('Generating C Code')
function_gen_mat(Dq, 'Dq');
function_gen_mat(Cqqd, 'Cqqd');
function_gen_mat(sym(Lqd), 'Lqd');
function_gen_mat(Nq, 'Nq');
function_gen_mat(Mm, 'Mm');

function_gen_mat(Jj1, 'Jj1');
function_gen_mat(Jj2, 'Jj2');
function_gen_mat(Je, 'Je');

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

vy0 = zeros(6,1); % bvx, bvy, bvz, w1, w2, w3,
%gdy0 = [0.1;0;0;0;0;0]; % bvx, bvy, bvz, w1, w2, w3,

% r0 = [0;0]; %r1, r2
r0 = [pi/2;0]; %r1, r2

rd0 = [0;0]; %r1d, r2d

y0 = [gy0(:); r0; vy0; rd0];

%Define States

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
% [qdd1, qdd2, qdd3]
%u = [0.1; 0.1; 0.1];

% Input accelerations
ua = [0;0;9.80665;0;0;0;0;0];

disp('Running Simulation')

%[ts,ys] = ode45(@(tfb,yfb)mantis_run(tfb, yfb, u, Dq_eq, Cqqd_eq, Kqd, N_eq, tspan, do_control),tspan,y0);

%t = ts:dt:te;
t = [dt,2*dt];
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

    %L = Lqd;

    %for i = 1:numel(N)
    %    %g, r1, r2
    %    %gb1_1,gb2_1,gb3_1,gb4_1,gb1_2,gb2_2,gb3_2,gb4_2,gb1_3,gb2_3,gb3_3,gb4_3,gb1_4,gb2_4,gb3_4,gb4_4,r1,r2
    %    N(i) = N_eq{i}(gk(3,1), gk(3,2), gk(3,3), rk(1), rk(2));
    %end

    M = double(Mm_sub);

    % Inverse Dynamics control
    %tau = D*ua + (C + L)*vk + N;
    tau = D*ua + C*vk;
    u(:,k) = M*tau;

    % Simulate 1 time step
    %y(:,k+1)= mantis_run(dt, y(:,k), D, C, L, N);

    y(:,k+1) = y(:,k);

    disp([num2str(100*(k/length(t))), '%'])
end
disp('100%')

disp('tau:')
disp(tau)
disp('u:')
disp(u)


% Render

figure(1);

for k=1:length(t)
    gk = reshape(y(1:16,k),[4,4]);
    rk = y(17:18,k);
    fl = params.frame.motor_radius;

    hold off; % Clean the display area
    plot3([0,0.5], [0,0], [0,0], 'color', 'r', 'linewidth', 1);
    hold on;
    plot3([0,0], [0,0.5], [0,0], 'color', 'g', 'linewidth', 1);
    plot3([0,0], [0,0], [0,0.5], 'color', 'b', 'linewidth', 1);

    %Calculate endpoint poses
    G = cell(size(G_eq));

    for i = 1:numel(G)
        %g, r1, r2
        %gb1_1,gb2_1,gb3_1,gb4_1,gb1_2,gb2_2,gb3_2,gb4_2,gb1_3,gb2_3,gb3_3,gb4_3,gb1_4,gb2_4,gb3_4,gb4_4,r1,r2
        G{i} = zeros(size(G_eq{i}));

        for j = 1:numel(G_eq{i})
            G{i}(j) = G_eq{i}{j}(gk(1,1), gk(1,2), gk(1,3), gk(1,4), ...
                                 gk(2,1), gk(2,2), gk(2,3), gk(2,4), ...
                                 gk(3,1), gk(3,2), gk(3,3), gk(3,4), ...
                                 rk(1), rk(2));
        end
    end

    %Plot frame
    for i = 1:params.motor.num
        ra = params.frame.map(i);
        len_a = params.frame.motor_radius;

        ga = [cos(ra), -sin(ra), 0, len_a*cos(ra); ...
              sin(ra),  cos(ra), 0, len_a*sin(ra); ...
                    0,        0, 1,             0; ...
                    0,        0, 0,             1];

        pa = G{1}*ga;

        plot3([G{1}(1,4),pa(1,4)], [G{1}(2,4),pa(2,4)], [G{1}(3,4),pa(3,4)], 'color', 'k', 'linewidth', 2);
    end

    %Plot arm links
    for i = 1:(numel(G)-1)
        plot3([G{i}(1,4),G{i+1}(1,4)], [G{i}(2,4),G{i+1}(2,4)], [G{i}(3,4),G{i+1}(3,4)], 'color', 'b', 'linewidth', 2);
    end

    %Draw figure
    ax_s = params.plot.size / 2;

    axis([-ax_s, ax_s, -ax_s, ax_s, 0, 2*ax_s]);
    axis('square')

    drawnow;

    disp([num2str(100*(k/length(t))), '%'])
end







