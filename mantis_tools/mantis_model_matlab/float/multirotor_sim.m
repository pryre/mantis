%% Cleanup
close all;
clear;
clc;


%% Setup Parameters
% Simulation parameters
ts = 0;
te = 1;
dt = 0.005;

k = 1; % number of links

% Load system parameters
%params = mantis_params('params.yaml');


%% Generate the system model

disp('Generating Parameters')

% Forward Kinematic Model
% Parameters
syms g la km kt 'positive'

mk = sym('m',[k,1], 'real');

% Base Position
x0 = sym('x0_',[3,1], 'real');
xd0 = sym('xd0_',[3,1], 'real');
xdd0 = sym('xdd0_',[3,1], 'real');
% Base Rotation (SO(3) defined as a rotation matrix)
r0 = sym('r0_',[3,3], 'real');
rd0 = sym('rd0_',[3,3], 'real');
rdd0 = sym('rdd0_',[3,3], 'real');
% Base Rotation (SO(3) defined as a quaternion)
% r0 = sym('r0_',[4,1], 'real');
% rd0 = sym('rd0_',[4,1], 'real');
% rdd0 = sym('rdd0_',[4,1], 'real');
% Joint Rotations
qh = sym('qh_',[k-1,1], 'real');
qhd = sym('qhd_',[k-1,1], 'real');
qhdd = sym('qhdd_',[k-1,1], 'real');

q = [x0(:); r0(:); qh(:)];
qd = [xd0(:); rd0(:); qhd(:)];
qdd = [xdd0(:); rdd0(:); qhdd(:)];

%% Rotation Jacobians

% Rotation Matrix example 
% R0 = r0; % for a rotation matrix

% Quaternion example
% syms r0_a r0_b r0_c r0_d 'real';
% r0 = [r0_a;r0_b;r0_c;r0_d];
% rho_r0 = [2*(r0_a^2 + r0_b^2) - 1, 2*(r0_b*r0_c - r0_a*r0_d), 2*(r0_b*r0_d + r0_a*r0_c); ...
%           2*(r0_b*r0_c + r0_a*r0_d), 2*(r0_a^2 + r0_c^2) - 1, 2*(r0_c*r0_d - r0_a*r0_b); ...
%           2*(r0_b*r0_d - r0_a*r0_c), 2*(r0_c*r0_d + r0_a*r0_b), 2*(r0_a^2 + r0_d^2) - 1];

% Base Link
R0 = r0;
% R0 = [2*(r0(1)^2 + r0(2)^2) - 1, 2*(r0(2)*r0(3) - r0(1)*r0(4)), 2*(r0(2)*r0(4) + r0(1)*r0(3)); ...
%       2*(r0(2)*r0(3) + r0(1)*r0(4)), 2*(r0(1)^2 + r0(3)^2) - 1, 2*(r0(3)*r0(4) - r0(1)*r0(2)); ...
%       2*(r0(2)*r0(4) - r0(1)*r0(3)), 2*(r0(3)*r0(4) + r0(1)*r0(2)), 2*(r0(1)^2 + r0(4)^2) - 1];

pd_R0_1 = sym(zeros(3,numel(r0)));
pd_R0_2 = sym(zeros(3,numel(r0)));
pd_R0_3 = sym(zeros(3,numel(r0)));

for i = 1:numel(r0)
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

Jtk1_rot = sym(zeros(3,numel(r0)));
for i = 1:numel(r0)
    Jtk1_rot(:,i) = diff(R0*ck{1}, r0(i));
end
Jtk{1} = [eye(3), Jtk1_rot, R0*Jht0];

Jlrk{1} = [zeros(3), R0'*R0*Jb_w0, Jhlr0];
Jgrk{1} = R0*Jlrk{1};

% DO OTHER LINKS HERE


%% Inertial Calculations

I = cell(k,1);

IT0x = (0.05^2 + la^2)/12;
IT0y = (0.05^2 + la^2)/12;
IT0z = la^2/2;

I{1} = [IT0x,    0,    0; ...
           0, IT0y,    0; ...
           0,    0, IT0z];


%% Dynamics

M = sym(zeros(numel(q)));
N = sym(zeros(numel(q)));

for i = 1:k
    Ick = inertial_offset(I{k},mk(k),ck{k});
    M = M + Jtk{k}'*mk*Jtk{k} + Jlrk{k}'*Ick*Jlrk{k};
    
    N = N + 0;
end








