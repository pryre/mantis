%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
time_start = 0;
time_end = 1;
time_dt = 0.01;

%g = -9.80665; %m/s

% Load system parameters
params = mantis_params('params.yaml');


%% Generate the system model
% Model is for a locked frame multirotor, such that only it can only pitch,
% but cannot move in any other rotation or translation
syms l1 l2 l3 lc2 lc3 q1 q2 q3 g m1 m2 m3 I1 I2 I3 'real'
syms qd1 qd2 qd3 qdd1 qdd2 qdd3 'real'

q = [q1; q2; q3];
qd = [qd1; qd2; qd3];
qdd = [qdd1; qdd2; qdd3];

qm = q1 - pi/2; %rotation to the mount

% Formulate D(q)
% Translational Kinetic Energy
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
rke1 = I1*[1, 0, 0; ...
           0, 0, 0; ...
           0, 0, 0];
rke2 = I2*[1, 1, 0; ...
           1, 1, 0; ...
           0, 0, 0];
rke3 = I3*[1, 1, 1; ...
           1, 1, 1; ...
           1, 1, 1];

% Total Inertia Matrix
tke = tke1 + tke2 + tke3;
rke = rke1 + rke2 + rke3;
Dq = tke + rke;

% Cristoffel Symbols
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
c212 = c121;
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

% Potential Energy
P1 = 0;
P2 = m2*g*(l1*sin(qm) + lc2*sin(qm+q2));
P3 = m3*g*(l1*sin(qm) + l2*sin(qm+q2) + lc3*sin(qm+q2+q3));
P = P1 + P2 + P3;

phi1 = diff(P,q1);
phi2 = diff(P,q2);
phi3 = diff(P,q3);

phi = simplify([phi1; phi2; phi3]);

% Coriolis Matrix
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

% Differential Equations
%tau1 = Dq(1,:)*qdd + Cqqd(1,:)*qd + phi(1);
%tau2 = Dq(2,:)*qdd + Cqqd(2,:)*qd + phi(2);
%tau3 = Dq(3,:)*qdd + Cqqd(3,:)*qd + phi(3);

tau = Dq*qdd + Cqqd*qd + phi;

%Define States
x = [q; qd; qdd];

sub_vals = [l1, 0.1; ...
            l2, 0.25; ...
            l3, 0.25; ...
            lc2, 0.25/2; ...
            lc3, 0.25/2; ...
            g, -9.80665; ...
            m1, 2.0; ...
            m2, 0.2; ...
            m3, 0.2];
        
tau_sub =  subs(tau, sub_vals(:,1), sub_vals(:,2));

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

% Simulation time
tspan = time_start:time_dt:time_end;

% Initial state
% [ x; y; z; dx; dy; dz; ...
%   phi; theta; psi; dphi; dtheta; dpsi; ...
%   thetal1; thetal2; dthetal1; dthetal2 ]

%y0 = [ 0; 0; 0; 0; 0; 0; ...
%       0; 0; 0; 0; 0; 0; ...
%       0; 0; 0; 0];

% Final state
%yf = mantis_goal([0,0,1], [0.2,0,0.6], params);

% Controller input
%u = -K*yf;
u = 0;

% Run the model
%[t,y] = ode45(@(t,y)mantis_run(y, u, params),tspan,y0);


%% Render

%figure(1);

%for k=1:length(t)
%    mantis_draw(y(:,k), params);
%    break;
    %pause(time_dt)
%end







