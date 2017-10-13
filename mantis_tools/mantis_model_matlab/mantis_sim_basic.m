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
% Model is for a locked frame multirotor, such that only it can only pitch,
% but cannot move in any other rotation or translation
syms l1 l2 l3 lc2 lc3 q1 q2 q3 g m1 m2 m3 Iz1 Iz2 Iz3 'real'
syms px pxd pxdd py pyd pydd pz pzd pzdd or ord ordd op opd opdd oy oyd oydd b1 b1d b1dd b2 b2d b2dd 'real'

qq = [px; py; pz; or; op; oy];
qqd = [pxd; pyd; pzd; ord; opd; oyd];
qqdd = [pxdd; pydd; pzdd; ordd; opdd; oydd];
qm = [b1; b2];
qmd = [b1d; b2d];
qmdd = [b1dd; b2dd];

q = [qq; qm];
qd = [qqd; qmd];
qdd = [qqdd; qmdd];


%% Forward Kinematic Model

% x_ = h1 + h2i + h3j + h4k + epsilon(h5 + h6i + h7j + h8k)
% P(x_) = h1 + h2i + h3j + h4k
% D(x_) = h5 + h6i + h7j + h8k
% x = [h1; h2; h3; h4; h5; h6; h7; h8];
% pos = 

rx = [1, 0, 0;
      0, cos(or), -sin(or);
      0, sin(or), cos(or)];
ry = [cos(op), 0, sin(op);
      0, 1, 0;
      -sin(op), 0, cos(op)];
rz = [cos(oy), -sin(oy), 0;
      sin(oy), cos(oy), 0;
      0, 0, 1];
  
pq = [px; py; pz];
rq = rz*ry*rx;

vec_xq = rq + (1/2)*pq*rq;

%%
% Formulate D(q)
% Translational Kinetic Energy
disp('Calculating Translational Kinetic Energy')
Jvc1 = [0, 0, 0; ...
        0, 0, 0; ...
        0, 0, 0];
Jvc2 = [-l1*sin(qm) - lc2*sin(qm+q2), -lc2*sin(qm+q2), 0; ...
        l1*cos(qm) + lc2*cos(qm+q2), lc2*cos(qm+q2), 0; ...
        0, 0, 0];
Jvc3 = [-l1*sin(qm) -  l2*sin(qm+q2) - lc3*sin(qm+q2+q3), -l2*sin(qm+q2) - lc3*sin(qm+q2+q3), -lc3*sin(qm+q2+q3); ...
        l1*cos(qm) + l2*cos(qm+q2) + lc3*cos(qm+q2+q3), l2*cos(qm*q2) + lc3*cos(qm+q2+q3), lc3*cos(qm+q2+q3); ...
        0, 0, 0];

tke1 = m1*((Jvc1')*Jvc1);
tke2 = m2*((Jvc2')*Jvc2);
tke3 = m3*((Jvc3')*Jvc3);

% Rotational Kinetic Energy
disp('Calculating Rotational Kinetic Energy')
rke1 = Iz1*[1, 0, 0; ...
           0, 0, 0; ...
           0, 0, 0];
rke2 = Iz2*[1, 1, 0; ...
           1, 1, 0; ...
           0, 0, 0];
rke3 = Iz3*[1, 1, 1; ...
           1, 1, 1; ...
           1, 1, 1];

% Total Inertia Matrix
disp('Calculating Inertia Matrix')
tke = tke1 + tke2 + tke3;
rke = rke1 + rke2 + rke3;
Dq = tke + rke;

% Cristoffel Symbols
disp('Calculating Cristoffel Symbols')
%cijk= (1/2)*(diff(Dq(k,j), qi) + diff(Dq(k,i), qj) + diff(Dq(i,j), qk));
%cijk=cjik
%q1
c111 = (1/2)*(diff(Dq(1,1), q1) + diff(Dq(1,1), q1) - diff(Dq(1,1), q1));
c121 = (1/2)*(diff(Dq(1,2), q1) + diff(Dq(1,1), q2) - diff(Dq(1,2), q1));
c131 = (1/2)*(diff(Dq(1,3), q1) + diff(Dq(1,1), q3) - diff(Dq(1,3), q1));
c211 = c121;
c221 = (1/2)*(diff(Dq(1,2), q2) + diff(Dq(1,2), q2) - diff(Dq(2,2), q1));
c231 = (1/2)*(diff(Dq(1,3), q2) + diff(Dq(1,2), q3) - diff(Dq(2,3), q1));
c311 = c131;
c321 = c231;
c331 = (1/2)*(diff(Dq(1,3), q3) + diff(Dq(1,3), q3) - diff(Dq(3,3), q1));
%q2
c112 = (1/2)*(diff(Dq(2,1), q1) + diff(Dq(2,1), q1) - diff(Dq(1,1), q2));
c122 = (1/2)*(diff(Dq(2,2), q1) + diff(Dq(2,1), q2) - diff(Dq(1,2), q2));
c132 = (1/2)*(diff(Dq(2,3), q1) + diff(Dq(2,1), q3) - diff(Dq(1,3), q2));
c212 = c122;
c222 = (1/2)*(diff(Dq(2,2), q2) + diff(Dq(2,2), q2) - diff(Dq(2,2), q2));
c232 = (1/2)*(diff(Dq(2,3), q2) + diff(Dq(2,2), q3) - diff(Dq(2,3), q2));
c312 = c132;
c322 = c232;
c332 = (1/2)*(diff(Dq(2,3), q3) + diff(Dq(2,3), q3) - diff(Dq(3,3), q2));
%q3
c113 = (1/2)*(diff(Dq(3,1), q1) + diff(Dq(3,1), q1) - diff(Dq(1,1), q3));
c123 = (1/2)*(diff(Dq(3,2), q1) + diff(Dq(3,1), q2) - diff(Dq(1,2), q3));
c133 = (1/2)*(diff(Dq(3,3), q1) + diff(Dq(3,1), q3) - diff(Dq(1,3), q3));
c213 = c123;
c223 = (1/2)*(diff(Dq(3,2), q2) + diff(Dq(3,2), q2) - diff(Dq(2,2), q3));
c233 = (1/2)*(diff(Dq(3,3), q2) + diff(Dq(3,2), q3) - diff(Dq(2,3), q3));
c313 = c133;
c323 = c233;
c333 = (1/2)*(diff(Dq(3,3), q3) + diff(Dq(3,3), q3) - diff(Dq(3,3), q3));

%c_col(i,j,k)
c_sym(1,1,1) = c111;
c_sym(1,2,1) = c121;
c_sym(1,3,1) = c131;
c_sym(2,1,1) = c211;
c_sym(2,2,1) = c221;
c_sym(2,3,1) = c231;
c_sym(3,1,1) = c311;
c_sym(3,2,1) = c321;
c_sym(3,3,1) = c331;
c_sym(1,1,2) = c112;
c_sym(1,2,2) = c122;
c_sym(1,3,2) = c132;
c_sym(2,1,2) = c212;
c_sym(2,2,2) = c222;
c_sym(2,3,2) = c232;
c_sym(3,1,2) = c312;
c_sym(3,2,2) = c322;
c_sym(3,3,2) = c332;
c_sym(1,1,3) = c113;
c_sym(1,2,3) = c123;
c_sym(1,3,3) = c133;
c_sym(2,1,3) = c213;
c_sym(2,2,3) = c223;
c_sym(2,3,3) = c233;
c_sym(3,1,3) = c313;
c_sym(3,2,3) = c323;
c_sym(3,3,3) = c333;

c_sym = simplify(c_sym);

% Coriolis Matrix
disp('Calculating Coriolis Matrix')
% TODO: Find a better way of making this matrix
Cqqd = [c_sym(1,1,1)*qd1 + c_sym(1,2,1)*qd2 + c_sym(1,3,1)*qd3, ...
        c_sym(2,1,1)*qd1 + c_sym(2,2,1)*qd2 + c_sym(2,3,1)*qd3, ...
        c_sym(3,1,1)*qd1 + c_sym(3,2,1)*qd2 + c_sym(3,3,1)*qd3; ...
        c_sym(1,1,2)*qd1 + c_sym(1,2,2)*qd2 + c_sym(1,3,2)*qd3, ...
        c_sym(2,1,2)*qd1 + c_sym(2,2,2)*qd2 + c_sym(2,3,2)*qd3, ...
        c_sym(3,1,2)*qd1 + c_sym(3,2,2)*qd2 + c_sym(3,3,2)*qd3; ...
        c_sym(1,1,3)*qd1 + c_sym(1,2,3)*qd2 + c_sym(1,3,3)*qd3, ...
        c_sym(2,1,3)*qd1 + c_sym(2,2,3)*qd2 + c_sym(2,3,3)*qd3, ...
        c_sym(3,1,3)*qd1 + c_sym(3,2,3)*qd2 + c_sym(3,3,3)*qd3];
    
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







