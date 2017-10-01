%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
time_start = 0;
time_end = 1;
time_dt = 0.01;

% Load system parameters
params = mantis_params('params.yaml');

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

y0 = [ 0; 0; 0; 0; 0; 0; ...
       0; 0; 0; 0; 0; 0; ...
       0; 0; 0; 0];

% Final state
yf = mantis_goal([0,0,2], [0.2,0,1.6], params);

% Controller input
%u = -K*yf;
u = 0;

% Run the model
%[t,y] = ode45(@(t,y)mantis_run(y, u, params),tspan,y0);


%% Render

figure(1);

% REMOVE THIS
t = tspan;
y = zeros(length(y0), length(t));

for i=1:length(t)
    y(:,i) = yf;
end
% REMOVE THIS

for k=1:length(t)
    mantis_draw(y(:,k), params);
    break;
    %pause(time_dt)
end







