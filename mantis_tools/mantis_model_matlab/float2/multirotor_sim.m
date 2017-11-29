%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
ts = 0;
te = 1;
dt = 0.005;

k = 1; % number of bodies

% Load system parameters
%params = mantis_params('params.yaml');


%% Generate the states for system model

disp('Generating Parameters')

% Parameters
syms g la km kt 'positive'

mk = sym('m',[k,1], 'real');
syms I0x I0y I0z 'real';

% Time
syms t 'real'

% Base Position
syms x0 y0 z0 x0d y0d z0d x0dd y0dd z0dd 'real'
syms x0t(t) y0t(t) z0t(t)
p0 = [x0; y0; z0];
p0d = [x0d; y0d; z0d];
p0dd = [x0dd; y0dd; z0dd];
p0t = {x0t; y0t; z0t};
p0dt = diff(p0t,t);
p0ddt = diff(p0t,t,t);

names.p0 = {'x0';'y0';'z0'};
names.p0d = {'x0d';'y0d';'z0d'};
names.p0dd = {'x0dd';'y0dd';'z0dd'};
names.p0t = {'x0t';'y0t';'z0t'};
names.p0dt = {'x0dt';'y0dt';'z0dt'};
names.p0ddt = {'x0ddt';'y0ddt';'z0ddt'};

% Base Rotation (SO(3) defined as a quaternion)
syms qw0 qx0 qy0 qz0 qw0d qx0d qy0d qz0d qw0dd qx0dd qy0dd qz0dd 'real'
syms qw0t(t) qx0t(t) qy0t(t) qz0t(t)
r0 = [qw0; qx0; qy0; qz0];
r0d = [qw0d; qx0d; qy0d; qz0d];
r0dd = [qw0dd; qx0dd; qy0dd; qz0dd];
r0t = {qw0t; qx0t; qy0t; qz0t};
r0dt = diff(r0t,t);
r0ddt = diff(r0t,t,t);


names.r0 = {'qw0';'qx0';'qy0';'qz0'};
names.r0d = {'qw0d';'qx0d';'qy0d';'qz0d'};
names.r0dd = {'qw0dd';'qx0dd';'qy0dd';'qz0dd'};
names.r0t = {'qw0t';'qx0t';'qy0t';'qz0t'};
names.r0dt = {'qw0dt';'qx0dt';'qy0dt';'qz0dt'};
names.r0ddt = {'qw0ddt';'qx0ddt';'qy0ddt';'qz0ddt'};

% Joint Rotations
% syms qj1(t) qj2(t)
% qj = formula([qj1; qj2]);
% qjd = diff(qj,t);
% qjdd = diff(qj,t,t);
% 
% names_qj = {'qj1';'qj2'};
% names_qjd = {'qj1d';'qjd'};
% names_qjdd = {'qj1dd';'qjdd'};
qj = [];
qjd = [];
qjdd = [];

names_qj = {};
names_qjd = {};
names_qjdd = {};

% States
q = [p0; r0; qj];
qd = [p0d; r0d; qjd];
qdd = [p0dd; r0dd; qjdd];

% Model parameters
d = numel(r0); % dimensions of rotational parameters
m = numel(qj); % number additional of joints
ns = numel(q); % number of states (3+d+m)

%% Jacobians

disp('Calculating Jacobians')

% Rotation Matrix example 
% syms r0_a r0_b r0_c r0_d r0_e r0_f r0_g r0_h r0_i 'real';
% r0 = [r0_a,r0_b,r0_c; ...
%       r0_d,r0_e,r0_f; ...
%       r0_g,r0_h,r0_i];
% rho_r0 = r0; % for a rotation matrix

% Quaternion example
% syms r0_a r0_b r0_c r0_d 'real';
% r0 = [r0_a;r0_b;r0_c;r0_d];
% rho_r0 = [2*(r0_a^2 + r0_b^2) - 1, 2*(r0_b*r0_c - r0_a*r0_d), 2*(r0_b*r0_d + r0_a*r0_c); ...
%           2*(r0_b*r0_c + r0_a*r0_d), 2*(r0_a^2 + r0_c^2) - 1, 2*(r0_c*r0_d - r0_a*r0_b); ...
%           2*(r0_b*r0_d - r0_a*r0_c), 2*(r0_c*r0_d + r0_a*r0_b), 2*(r0_a^2 + r0_d^2) - 1];

% Base Link Position Jacobian
% R0x = [1,          0,           0; ...
%        0, cos(r0(1)), -sin(r0(1)); ...
%        0, sin(r0(1)), cos(r0(1))];
%    
% R0y = [ cos(r0(2)), 0, sin(r0(2)); ...
%                  0, 1,          0; ...
%        -sin(r0(2)), 0, cos(r0(2))];
% 
%    R0z = [cos(r0(3)), -sin(r0(3)), 0; ...
%        sin(r0(3)),  cos(r0(3)), 0; ...
%                 0,           0, 1];
% R0 = R0z*R0y*R0x;
%
% R0 = [1 - 2*(r0(3)^2 + r0(4)^2), 2*(r0(2)*r0(3) - r0(4)*r0(1)), 2*(r0(2)*r0(4) + r0(3)*r0(1)); ...
%       2*(r0(2)*r0(3) + r0(4)*r0(1)), 1 - 2*(r0(2)^2 + r0(4)^2), 2*(r0(3)*r0(4) - r0(2)*r0(1)); ...
%       2*(r0(2)*r0(4) - r0(3)*r0(1)), 2*(r0(3)*r0(4) + r0(2)*r0(1)), 1 - 2*(r0(2)^2 + r0(3)^2)];
%
% R0 = r0;

R0 = [2*(r0(1)^2 + r0(2)^2) - 1, 2*(r0(2)*r0(3) - r0(1)*r0(4)), 2*(r0(2)*r0(4) + r0(1)*r0(3)); ...
      2*(r0(2)*r0(3) + r0(1)*r0(4)), 2*(r0(1)^2 + r0(3)^2) - 1, 2*(r0(3)*r0(4) - r0(1)*r0(2)); ...
      2*(r0(2)*r0(4) - r0(1)*r0(3)), 2*(r0(3)*r0(4) + r0(1)*r0(2)), 2*(r0(1)^2 + r0(4)^2) - 1];

pd_R0_1 = sym(zeros(3,d));
pd_R0_2 = sym(zeros(3,d));
pd_R0_3 = sym(zeros(3,d));

for i = 1:d
    pd_R0_1(:,i) = diff(R0(:,1), r0(i));
    pd_R0_2(:,i) = diff(R0(:,2), r0(i));
    pd_R0_3(:,i) = diff(R0(:,3), r0(i));
end

Jg_w0 = R0(:,3)*R0(:,2)'*pd_R0_1 + ...
        R0(:,2)*R0(:,1)'*pd_R0_3 + ...
        R0(:,1)*R0(:,3)'*pd_R0_2;

Jb_w0 = R0'*Jg_w0;


Jtk = cell(k,1);
Jlrk = cell(k,1);
Jgrk = cell(k,1);
ck = cell(k,1);

% Base Link
ck{1} = [0;0;0];
Jht0 = zeros(3,k-1);
Jhlr0 = zeros(3,k-1);
%Jhbr0 = zeros(3,k-1);

% Jtk1_rot = sym(zeros(3,d));
% for i = 1:numel(r0)
%     Jtk1_rot(:,i) = diff(R0*ck{1}, r0(i));
% end
% Jtk{1} = [eye(3), Jtk1_rot, R0*Jht0];

Jtk{1} = [eye(3), -vee_up(R0*ck{1})*Jg_w0, R0*Jht0];

Jlrk{1} = [zeros(3), R0'*R0*Jb_w0, Jhlr0];
% Jgrk{1} = R0*Jlrk{1};


% !!DO OTHER LINKS HERE!!


% Derivative Jacobians

Jtkd{1} = zeros(size(Jtk{1})); % !!FIX ME!!

Jlrkt{1} = subs(Jlrk{1},r0,r0t);
Jlrkdt{1} = sym(zeros(size(Jlrkt{1})));
for i = 1:numel(Jlrkt{1})
    Jlrkdt{1}(i) = diff(Jlrkt{1}(i),t);
end
Jlrkd{1} = subs(subs(Jlrkdt{1},r0dt,r0d),r0t,r0);


% !!DO OTHER LINKS HERE!!


%% Inertial Calculations

I = cell(k,1);

I{1} = [I0x,   0,   0; ...
          0, I0y,   0; ...
          0,   0, I0z];


%% Dynamics
disp('Building dynamics equations')

M = sym(zeros(ns));
N = sym(zeros(ns));

for i = 1:k
    Ick = inertial_offset(I{i},mk(i),ck{i});
    
    M = M + Jtk{i}'*mk*Jtk{i} + Jlrk{i}'*Ick*Jlrk{i};
    
    N = N + Jtk{i}'*mk*Jtkd{i} + Jlrk{i}'*Ick*Jlrkd{i} - Jlrk{i}'*vee_up(Ick*Jlrk{i}*qd)*Jlrk{i};
end


S = sym([zeros(d,3), eye(d), zeros(d,m)]);
A = eye(ns) - (S'*S*q)*q';

G = [0;0;-g;zeros(d+m,1)]; % Gravity state acceleration
U = [zeros(3,1); zeros(d,1); zeros(m,1)]; % Control input
F = zeros(ns,1); % Additional contact forces


disp('Symplifying dynamics equations')

disp('    Preparing M dynamics')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(M)) '\n    \n']);
parfor i = 1:numel(M)
    M(i) = simplify(M(i));
    fprintf('\b|\n');
end

% disp('    Preparing N dynamics')
% fprintf('    Progress:\n');
% fprintf(['    ' repmat('.',1,numel(N)) '\n    \n']);
% parfor i = 1:numel(N)
%     N(i) = simplify(N(i));
%     fprintf('\b|\n');
% end

disp('Formulated dynamics:');
disp('    A(M*qdd + N*qd) = A(M*G + U + F)');


%% De-Symbolize

disp('De-symbolizing Equations')

sub_vals = [g, 9.80665; ...
            km, 0; ...
            kt, 0; ...
            la, 0.275; ...
            mk(1), 2.0; ...
            I0x, (0.05^2 + la^2)/12; ...
            I0y, (0.05^2 + la^2)/12; ...
            I0z, la^2/2];

% Have to substitute it to account for redefined variables
sub_vals(:,2) = subs(sub_vals(:,2), sub_vals(:,1), sub_vals(:,2));

%substitute static parameters
M_sub = subs(M, sub_vals(:,1), sub_vals(:,2));
N_sub = subs(N, sub_vals(:,1), sub_vals(:,2));
G_sub = subs(G, sub_vals(:,1), sub_vals(:,2));
A_sub = subs(A, sub_vals(:,1), sub_vals(:,2));

% Prep equations
M_eq = cell(size(M));
N_eq = cell(size(N));
A_eq = cell(size(A));

M_state_vars = q;
N_state_vars = [q;qd];
A_state_vars = q;

disp('    Preparing M solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(M)) '\n    \n']);
parfor i = 1:numel(M)
    M_eq(i) = {matlabFunction(M_sub(i), 'Vars', M_state_vars)};
    fprintf('\b|\n');
end

disp('    Preparing N solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(N)) '\n    \n']);
parfor i = 1:numel(N)
    N_eq(i) = {matlabFunction(N_sub(i), 'Vars', N_state_vars)};
    fprintf('\b|\n');
end

disp('    Preparing A solver')
fprintf('    Progress:\n');
fprintf(['    ' repmat('.',1,numel(A)) '\n    \n']);
parfor i = 1:numel(A)
    A_eq(i) = {matlabFunction(A_sub(i), 'Vars', A_state_vars)};
    fprintf('\b|\n');
end


%% Prepare Simulation
disp('Preparing Simulation')

% Initial State
phi0 = 0;
theta0 = 0;
psi0 = 0;
x = 0;
y = 0;
z = 1.0;

py0 = [x; y; z];
%Ry0 = [0,0,0];
Ry0 = angle2quat(psi0, theta0, phi0);
%Ry0 = angle2dcm(psi0, theta0, phi0);
q0 = [py0(:);Ry0(:)];

qd0 = zeros(size(q0));

y0 = [q0(:); qd0(:)];

% Final State
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
% [qdd1, qdd2, qdd3]
%u = [0.1; 0.1; 0.1];
u = zeros(size(6));


%% Run Simulation

disp('Running Simulation')

%[ts,ys] = ode45(@(tfb,yfb)mantis_run(tfb, yfb, u, Dq_eq, Cqqd_eq, Kqd, N_eq, tspan, do_control),tspan,y0);

t = ts:dt:te;
y = zeros(length(y0), length(t));
y(:,1) = y0;

for i = 1:1%(length(t)-1)
    yi = reshape(y(:,i), [length(y(:,i))/2,2]);
    qi = yi(:,1);
    qdi = yi(:,2);
    
    % Prep dynamics matricies
    Mq = zeros(size(M_eq));
    Nqqd = zeros(size(N_eq));
    Aq = zeros(size(A_eq));
    
    for j = 1:numel(M_eq)
        Mq(j) = M_eq{j}(qi(1),qi(2),qi(3), ...
                        qi(4),qi(5),qi(6), ...
                        qi(7));
    end

    for j = 1:numel(N_eq)
        Nqqd(j) = N_eq{j}(qi(1),qi(2),qi(3), ...
                          qi(4),qi(5),qi(6), ...
                          qi(7), ...
                          qdi(1),qdi(2),qdi(3), ...
                          qdi(4),qdi(5),qdi(6), ...
                          qdi(7));
    end
    
    Gc = double(G_sub); % gravity constant acceleration
    
    for j = 1:numel(A_eq)
        Aq(j) = A_eq{j}(qi(1),qi(2),qi(3), ...
                        qi(4),qi(5),qi(6), ...
                        qi(7));
    end
    
    Fc = F; % Additional contact forces
    
    % Simulate 1 time step
    y(:,i+1)= multirotor_run(dt, y(:,i), Mq, Nqqd, Gc, Fc, Aq);
    
    disp([num2str(100*(i/length(t))), '%'])
end
disp('100%')






