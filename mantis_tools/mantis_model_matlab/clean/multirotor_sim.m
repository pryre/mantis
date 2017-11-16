%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
ts = 0;
te = 1;
dt = 0.005;

%g = -9.80665; %m/s

% Load system parameters
params = mantis_params('params.yaml');


%% Generate the system model

disp('Generating Parameters')

% Forward Kinematic Model
%Parameters
syms g la km kt m0 'positive'
syms phi bw1 bw1d theta bw2 bw2d psi bw3 bw3d 'real'
syms bx by bz bvx bvxd bvy bvyd bvz bvzd 'real'
%%
% Base Link
g0 = sym('g0', 4, 'real');
g0inv = [g0(1:3,1:3)', -g0(1:3,1:3)'*g0(1:3,4); ...
         zeros(1,3), 1];

q = [phi; theta; psi; bx; by; bz];
qd = [bw1; bw2; bw3; bvx; bvy; bvz];
qdd = [bw1d; bw2d; bw3d; bvxd; bvyd; bvzd];
     
%% Equations of Motion

disp('Defining Inertials')
% Define inertials

syms IT0x IT0y IT0z 'positive'

% Base link
IJ0 = [IT0x, 0, 0; ... %Rotational Inertial Tensor
       0, IT0y, 0; ...
       0, 0, IT0z];
IT0 = diag(ones(3,1)); %Translational Inertial
I0 = [m0*IJ0, zeros(3); ... %Pose Inertial Tensor
      zeros(3), m0*IT0];

% Formulate M(r)

disp('Calculating Inertia Matrix')
M_A_A = I0;

%Same as M(r)
Dq = M_A_A;
  
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


%% Potential Energy
disp('Calculating Potential Energy')

% Gravity effect on links

P = zeros(size(q));
P = sym(P);

Pgrav = [0;0;g0(3,4)];
Rgrav = g0(1:3,1:3);
f_grav = Rgrav'*Pgrav;
%f_grav = Pgrav;
P(4) = bx*g*m0*f_grav(1); %bx
P(5) = by*g*m0*f_grav(2); %by
P(6) = bz*g*m0*f_grav(3); %bz

q(4:6) = [bx;by;bz];

P_sum = sum(P);

Nq = zeros(size(q));
Nq = sym(Nq);

for i = 1:length(q)
    Nq(i) = simplify(diff(P_sum,q(i)));
end

disp(Nq)


%% Friction Losses
disp('Calculating Friction Losses')
fric = 0.05;

%No losses
K1 = zeros(1,6);
K2 = zeros(1,6);
K3 = zeros(1,6);
K4 = zeros(1,6);
K5 = zeros(1,6);
K6 = zeros(1,6);

Lqd = [K1; K2; K3; K4; K5; K6];


%% Differential Equations
disp('Calculating Equations of Motion')
tau = Dq*qdd + (Cqqd + Lqd)*qd + Nq;

sub_vals = [km, 0; ...
            kt, 0; ...
            la, 0.275; ...
            m0, 2.0; ...
            g, 9.80665; ...
            IT0x, (0.05^2 + la^2)/12; ...
            IT0y, (0.05^2 + la^2)/12; ...
            IT0z, la^2/2];
        
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


%% Prep equations for faster calculations

Dq_sub = subs(Dq, sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(Cqqd, sub_vals(:,1), sub_vals(:,2));
Nq_sub = subs(Nq, sub_vals(:,1), sub_vals(:,2));
Dq_sub = subs(Dq_sub, sub_vals(:,1), sub_vals(:,2));
Cqqd_sub = subs(Cqqd_sub, sub_vals(:,1), sub_vals(:,2));
Nq_sub = subs(Nq_sub, sub_vals(:,1), sub_vals(:,2));


%disp('Testing Dq Inverse')
%inv_test = inv(Dq_sub);

%if ~inv_test
%    disp(inv_test);
%    error('Dq cannot be inversed!')
%end

Dq_eq = cell(size(Dq));
Cqqd_eq = cell(size(Cqqd));
N_eq = cell(size(Nq));


state_vars = qd;
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
fprintf(['    ' repmat('.',1,numel(Nq)) '\n    \n']);
parfor i = 1:numel(Nq)
    N_eq(i) = {matlabFunction(Nq_sub(i), 'Vars', g0(:))};
    fprintf('\b|\n');
end

%% Run Simulation
disp('Preparing Simulation')

phi0 = pi/4;
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

%gdy0 = zeros(6,1); % w1, w2, w3, bvx, bvy, bvz
gdy0 = [1;0;0;0;0;0]; % w1, w2, w3, bvx, bvy, bvz

y0 = [gy0(:); gdy0];
         
%Define States

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
% [qdd1, qdd2, qdd3]
%u = [0.1; 0.1; 0.1];
u = zeros(size(6));

%%
disp('Running Simulation')

%[ts,ys] = ode45(@(tfb,yfb)mantis_run(tfb, yfb, u, Dq_eq, Cqqd_eq, Kqd, N_eq, tspan, do_control),tspan,y0);

t = ts:dt:te;
y = zeros(length(y0), length(t));
y(:,1) = y0;

for k = 1:(length(t)-1)
    gk = reshape(y(1:16,k),[4,4]);
    vk = y(17:22,k);
    
    D = zeros(size(Dq_eq));
    C = zeros(size(Cqqd_eq));
    N = zeros(size(N_eq));
    
    for i = 1:numel(Dq_eq)
                        %bw1, bw2, bw3, bvx, bvy, bvz
        D(i) = Dq_eq{i}(vk(1), vk(2), vk(3), vk(4), vk(5), vk(6));
    end

    for i = 1:numel(C)
                          %bw1, bw2, bw3, bvx, bvy, bvz
        C(i) = Cqqd_eq{i}(vk(1), vk(2), vk(3), vk(4), vk(5), vk(6));
    end
    
    L = Lqd;
   
    for i = 1:numel(N)
        %g, r1, r2
        %g01_1,g02_1,g03_1,g04_1,g01_2,g02_2,g03_2,g04_2,g01_3,g02_3,g03_3,g04_3,g01_4,g02_4,g03_4,g04_4,r1,r2
        N(i) = N_eq{i}(gk(1,1), gk(2,1), gk(3,1), gk(4,1), ...
                       gk(1,2), gk(2,2), gk(3,2), gk(4,2), ...
                       gk(1,3), gk(2,3), gk(3,3), gk(4,3), ...
                       gk(1,4), gk(2,4), gk(3,4), gk(4,4));
    end
    
    % Simulate 1 time step
    y(:,k+1)= multirotor_run(dt, y(:,k), D, C, L, N);
    
    disp([num2str(100*(k/length(t))), '%'])
end
disp('100%')


%% Render

figure(1);

for k=1:length(t)
    %% Prep
    gk = reshape(y(1:16,k),[4,4]);
    fl = params.frame.motor_arm_length;
    al = params.arm.length;
    
    hold off; % Clean the display area
    plot3([0,0.5], [0,0], [0,0], 'color', 'r', 'linewidth', 1);
    hold on;
    plot3([0,0], [0,0.5], [0,0], 'color', 'g', 'linewidth', 1);
    plot3([0,0], [0,0], [0,0.5], 'color', 'b', 'linewidth', 1);
    
    
    %% Base
    
    %frame_base = [eye(3), [x;y;z]; zeros(1,3), 1];
    gf1 = [eye(3), [fl;0;0]; zeros(1,3), 1];
    gf2 = [eye(3), [-fl;0;0]; zeros(1,3), 1];
    gf3 = [eye(3), [0;fl;0]; zeros(1,3), 1];
    gf4 = [eye(3), [0;-fl;0]; zeros(1,3), 1];
    
    pf1 = gk*gf1;
    pf2 = gk*gf2;
    pf3 = gk*gf3;
    pf4 = gk*gf4;
    
    plot3([gk(1,4),pf1(1,4)], [gk(2,4),pf1(2,4)], [gk(3,4),pf1(3,4)], 'color', 'k', 'linewidth', 2);
    plot3([gk(1,4),pf2(1,4)], [gk(2,4),pf2(2,4)], [gk(3,4),pf2(3,4)], 'color', 'k', 'linewidth', 2);
    plot3([gk(1,4),pf3(1,4)], [gk(2,4),pf3(2,4)], [gk(3,4),pf3(3,4)], 'color', 'k', 'linewidth', 2);
    plot3([gk(1,4),pf4(1,4)], [gk(2,4),pf4(2,4)], [gk(3,4),pf4(3,4)], 'color', 'k', 'linewidth', 2);
    
    
    %% Draw
    
    ax_s = params.plot.size / 2;
    
    axis([-ax_s, ax_s, -ax_s, ax_s, 0, 2*ax_s]);
    axis('square')
    
    drawnow;
    
    %%
    disp([num2str(100*(k/length(t))), '%'])
    %% Old
% [ x; y; z; dx; dy; dz; ...
%   phi; theta; psi; dphi; dtheta; dpsi; ...
%   thetal1; thetal2; dthetal1; dthetal2 ]
%     viz = [0;0;1;0;0;0; ...
%            0;-ys(k,1);0;0;-ys(k,2);0; ...
%            ys(k,3);ys(k,5);ys(k,4);ys(k,6)];
%     mantis_draw(viz, params);
    %pause(time_dt)
end







