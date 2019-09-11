%% Setup

close all;
clear;
clc;

set(0,'defaultTextInterpreter','latex');

% https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2017/RD_HS2017script.pdf
% Pg. 76
% Hence, the eigenfrequency and a dimensionless damping value of the system 
% are given by:
% ω=√kp,(3.78)
% D=kd/2√kp.(3.79)
% Critical damping is achieved for D=1, overcritical damping for D>1 and 
% un-dercritical  damping  forD <1. The  compliance  of  the  controller 
% can  be  adjustedby varyingkp.  For example, assuming that the time 
% constant respectively oscillationfrequency around the nominal point 
% should be 3 Hz, the ideal control gainkpis 350.Furthermore, critical 
% damping requireskd= 37. This holds as good starting values for controller 
% gain tuning.

%% User Variables
% Spline Order:
%   cubic:  3
%   quntic: 5
%   septic: 7
%   nonic:  9
order = 9;

t0 = 0;
dt = 1/500;
tf = 10;

num_via_points = 9;

% 'x_only'
% 'y_only'
% 'z_only'
% 'yaw_only'
% 'circle_flat'
% 'circle_flat_yaw'
% 'circle_raised'
% 'circle_raised_yaw'
tname = 'circle_raised_yaw';

% via_est_method = 'linear';
via_est_method = 'fdcc';

mass_c.m = 1.83497;
mass_c.Ixx = 0.02961;
mass_c.Iyy = 0.02961;
mass_c.Izz = 0.05342;

mass_d.m = 1.83497;
mass_d.Ixx = 0.02961;
mass_d.Iyy = 0.02933;
mass_d.Izz = 0.05342;

% KxP = 2.0;      % Position tracking P gain
% KxdP =  6.0;    % Velocity tracking P gain
% KtP = 60.0;     % Angular tracking P gain
% KtdP = 10.0;    % Angular Rate tracking P gain

% KxP = 1.0;      % Position tracking P gain
% KxdP =  2.0;    % Velocity tracking P gain
% KtP = 60.0;     % Angular tracking P gain
% KtdP = 10.0;    % Angular Rate tracking P gain

KxP = 0.0;     % Position tracking P gain
KxdP = 0.0;    % Velocity tracking P gain
KtP = 0.0;     % Angular tracking P gain
KtdP = 0.0;    % Angular Rate tracking P gain

yaw_w = 1.0;    % Yaw weighting for rotational tracking
theta_max = deg2rad(30); % Maximum thrust vectoring angle (from vertical)


%% Script Variables
disp('Generating...')

via_steps = ((tf - t0) / dt) / (num_via_points - 1);
num_traj_points = num_via_points - 1;
[vpx, vpy, vpz, vppsi] = trajectory_vias(tname, num_via_points);

% x0 = [[1;0;0;0];    %reshape(diag(ones(3,1)),9,1);  % Rotation (World)
%       [0;1;0];      % Position (World)
%       zeros(3,1);   % Linear Velocity (World)
%       zeros(3,1);   % Angular Velocity (Body)
%       zeros(3,1)];  % Linear Velocity (Body)

x0 = [eul2quat([vppsi(1),0,0])';    % Rotation (World)
% x0 = [eul2quat([pi/2,0,0])';    % Rotation (World)
% x0 = [eul2quat([0,pi,0])';    % Rotation (World)
      vpx(1);
      vpy(1);
      vpz(1);                       % Position (World)
      zeros(3,1);                   % Linear Velocity (World)
      zeros(3,1);                   % Angular Velocity (Body)
      zeros(3,1)];                  % Linear Velocity (Body)
      
STATE_QW = 1;
STATE_QX = 2;
STATE_QY = 3;
STATE_QZ = 4;
STATE_X = 5;
STATE_Y = 6;
STATE_Z = 7;
STATE_VX = 8;
STATE_VY = 9;
STATE_VZ = 10;
STATE_WX_B = 11;
STATE_WY_B = 12;
STATE_WZ_B = 13;
STATE_VX_B = 14;
STATE_VY_B = 15;
STATE_VZ_B = 16;

STATE_Q = STATE_QW:STATE_QZ;
STATE_XYZ = STATE_X:STATE_Z;
STATE_VXYZ = STATE_VX:STATE_VZ;
STATE_WXYZ_B = STATE_WX_B:STATE_WZ_B;
STATE_VXYZ_B = STATE_VX_B:STATE_VZ_B;
STATE_REDUCED = [STATE_WXYZ_B,STATE_VXYZ_B];
STATE_REDUCED_W_B = 1:3;
STATE_REDUCED_V_B = 4:6;

CONTROL_AX = 1;
CONTROL_AY = 2;
CONTROL_AZ = 3;
CONTROL_QW = 4;
CONTROL_QX = 5;
CONTROL_QY = 6;
CONTROL_QZ = 7;
CONTROL_R_E_X = 8;
CONTROL_R_E_Y = 9;
CONTROL_R_E_Z = 10;
CONTROL_WX_B = 11;
CONTROL_WY_B = 12;
CONTROL_WZ_B = 13;
CONTROL_DWX_B = 14;
CONTROL_DWY_B = 15;
CONTROL_DWZ_B = 16;

CONTROL_A = CONTROL_AX: CONTROL_AZ;
CONTROL_Q = CONTROL_QW:CONTROL_QZ;
CONTROL_R_E = CONTROL_R_E_X:CONTROL_R_E_Z;
CONTROL_WXYZ_B = CONTROL_WX_B:CONTROL_WZ_B;
CONTROL_DWXYZ_B = CONTROL_DWX_B:CONTROL_DWZ_B;

SPLINE_POS = 1;
SPLINE_VEL = 2;
SPLINE_ACC = 3;
SPLINE_JERK = 4;
SPLINE_SNAP = 5;

g = 9.80665;
g_vec = [0;0;-g];

t = t0:dt:tf;
% sd = t_vias(2) - t_vias(1);
t_vias = linspace(t0,tf,num_via_points);

% t_vias = [0, 1, 2.5, 3, 5, 6, 7, 9, 10];

if logical(mod(t_vias,dt))
    error('t_vias entries must be a multiple of dt')
end

vias_x = gen_vias(via_est_method, vpx, t_vias);
vias_y = gen_vias(via_est_method, vpy, t_vias);
vias_z = gen_vias(via_est_method, vpz, t_vias);
vias_psi = gen_vias(via_est_method, vppsi, t_vias);
% [st, s_x] = gen_spline(t0, tf, vias_x, order, via_steps);
% [~, s_y] = gen_spline(t0, tf, vias_y, order, via_steps);
% [~, s_z] = gen_spline(t0, tf, vias_z, order, via_steps);
% [~, s_psi] = gen_spline(t0, tf, vias_psi, order, via_steps);
[st, s_x] = gen_spline(t_vias, vias_x, order, dt);
[~, s_y] = gen_spline(t_vias, vias_y, order, dt);
[~, s_z] = gen_spline(t_vias, vias_z, order, dt);
[~, s_psi] = gen_spline(t_vias, vias_psi, order, dt);

if size(st) ~= size(t)
    error('Error constructing spline times')
end

% Build state vector
x = zeros(size(x0,1),length(t));
x(:,1) = x0;

% Build control vector
c = zeros(16,length(t));


%% Simulation
reverseStr = '';
progressLast = 0;

for i = 2:length(t)
    progress = 100*i/length(t);
    if progress > progressLast + 0.1
        msg = sprintf('Simulating... %3.1f', progress);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        progressLast = progress;
    end
    
    % Previous rotation matrix
    R_p = quat2rotm(x(STATE_Q,i-1)');
    
    % Spline reference vectors
    pos = [s_x(SPLINE_POS,i);s_y(SPLINE_POS,i);s_z(SPLINE_POS,i)];
    vel = [s_x(SPLINE_VEL,i);s_y(SPLINE_VEL,i);s_z(SPLINE_VEL,i)];
    acc = [s_x(SPLINE_ACC,i);s_y(SPLINE_ACC,i);s_z(SPLINE_ACC,i)] - g_vec; %XXX: -g to have a positive term for gravity added
    jerk = [s_x(SPLINE_JERK,i);s_y(SPLINE_JERK,i);s_z(SPLINE_JERK,i)];
    snap = [s_x(SPLINE_SNAP,i);s_y(SPLINE_SNAP,i);s_z(SPLINE_SNAP,i)];
    
    yaw = s_psi(SPLINE_POS,i);
    dyaw = s_psi(SPLINE_VEL,i);
    ddyaw = s_psi(SPLINE_ACC,i);
    
    % Build acceleration control reference from spline tracking error
    c(CONTROL_A,i) = acc + KxP*(pos - x(STATE_XYZ,i-1)) + KxdP*(vel - x(STATE_VXYZ,i-1));

    % Rotation control reference from acceleration vector with tracking error
    [R_sp_c, q_sp_c] = rot_from_vec_yaw(c(CONTROL_A,i),yaw);
    c(CONTROL_Q,i) = q_sp_c';
    
    % Rotation reference from acceleration vector without tracking error
%     [R_sp_s, q_sp_s] = rot_from_vec_yaw(acc,yaw);
%     
    % Calculate angle axis of acceleration vector to vertical
    avec_a_a = vrrotvec([0;0;1],c(CONTROL_A,i));
    theta_a = avec_a_a(4);
    if abs(theta_a) > theta_max
       warning('Large thrust vector detected')
    end
    
%     acc_n = acc / norm(acc);
%     j_tngt = jerk - dot(jerk,acc_n)*acc_n;
%     s_tngt = snap - dot(snap,acc_n)*acc_n;
    
%     w = cross(acc,j_tngt) / (norm(acc)^2);
%     w_d = cross(acc,s_tngt) / (norm(acc)^2);

%     w_dd = zeros(3,1);
%     w_dd(2) = -g*(2*s_x(3,i)*s_x(4,i)^2 - (s_x(3,i)^2 + g^2)*s_x(5,i)) / ((s_x(3,i)^2 + g^2)^2); %theta_dd
%     disp(['norm: ' num2str((cross(acc,snap) / (norm(acc)^2))')])
%     disp(['tngt: ' num2str((cross(acc,s_tngt) / (norm(acc)^2))')])

%     eul = quat2eul(q_sp_s, 'ZYX');
%     % phi = eul(3);
%     % theta = eul(2);
%     % psi = eul(1);
%     Lbi = [1,0,-sin(eul(2));
%            0, cos(eul(3)), sin(eul(3))*cos(eul(2));
%            0, -sin(eul(3)), cos(eul(3))*cos(eul(2))];

%     [w_sp,wd_sp] = map_body_rates_accels(R_sp_s'*acc, R_sp_s'*jerk, R_sp_s'*snap);
    
%     wed = Lbi*[0;0;dyaw];
%     wdedd = Lbi*[0;0;ddyaw];
%     wed = R_sp_s'*[0;0;dyaw];
%     wdedd = R_sp_s'*[0;0;ddyaw];

%     [we_sp,wed_sp] = map_yaw_rates_accels(R_sp_s, w_sp, dyaw, ddyaw);
%     R_sp_yaw = eul2rotm([yaw,0,0],'ZYX');
%     ed_sp = [0;0;dyaw];

%     c(CONTROL_WXYZ_B,i) = R_sp_yaw*w_sp + we_sp;
%     c(CONTROL_DWXYZ_B,i) = R_sp_yaw*vee_up(ed_sp)*w_sp + R_sp_yaw*wd_sp + wed_sp;
   
    [R_sp_s,w_sp,wd_sp] = map_angles_rates_accels(acc, jerk, snap, yaw, dyaw, ddyaw);
    c(CONTROL_WXYZ_B,i) = w_sp;
    c(CONTROL_DWXYZ_B,i) = wd_sp;
    
%     % Here we use the error_w_trans function for both w and wd
%     % However in the case of wd, it is more out of convinience to do the 
%     % SO(3) transport mapping easily
%     v_w = error_w_trans(c(CONTROL_DWXYZ_B,i), zeros(3,1), R_sp_s, R_p ) ...
%         + KtdP*error_w_trans(c(CONTROL_WXYZ_B,i), x(STATE_WXYZ_B,i-1), R_sp_s, R_p ) ...
%         + KtP*c(CONTROL_R_E,i);

%     % SO(3) transport mapping for w_sp and wd_sp 
%     [ew, ewd] = error_w_wd_trans(R_sp_s, R_p, ...
%                                  c(CONTROL_WXYZ_B,i), x(STATE_WXYZ_B,i-1), ...
%                                  c(CONTROL_DWXYZ_B,i), zeros(3,1));

    [eR_s, ew, ewd] = error_R_w_wd_trans(R_sp_s, R_p, ...
                                         c(CONTROL_WXYZ_B,i), x(STATE_WXYZ_B,i-1), ...
                                         c(CONTROL_DWXYZ_B,i));

    % Calculate a different attitude rotation error function for the
    % that applies the control for the position/velocity terms.
    c(CONTROL_R_E,i) = 0.5*vee_down(R_p'*R_sp_c - R_sp_c'*R_p);
%     c(CONTROL_R_E,i) = error_q_att(q_sp_c, x(STATE_Q,i-1)', yaw_w);
                             
    % SO(3) PD Tracking
    wd_b = ewd ...
         + KtdP*ew ...
         + KtP*c(CONTROL_R_E,i);
    
%     % The method below is the old/ignorant method, does not account for
%     % rotation error in the w/wd calculations
%     v_w = c(CONTROL_DWXYZ_B,i) ...
%         + KtdP*(c(CONTROL_WXYZ_B,i) - x(STATE_WXYZ_B,i-1)) ...
%         + KtP*c(CONTROL_R_E,i);

    z_acc = norm(c(CONTROL_A,i));

    % Inversion protection
%     if sign_no_zero(c(CONTROL_AZ,i)) ~= sign_no_zero(R_p(3,3))
%         %warning('Detected unexpected inversion, zeroing throttle')
%         z_acc = 0;
%     end

%     % Calculate angle axis of z basis to acceleration vector
%     avec_r_a = vrrotvec(R_p(:,3),c(CONTROL_A,i));
%     theta_r = avec_r_a(4);
% %     if abs(theta) > deg2rad(22.5)
% %        warning('Large thrust difference detected')
% %        z_acc = 0;
% %     end
%     if (z_acc < 0) || (theta_r > pi/2)
% %        warning('Large thrust difference or reverse thrust detected')
%        z_acc = 0;
%     else
%         % Other options would be to set the value to zero over a set
%         % cut-off (e.g. pi/8) which would produce a stepping effect.
%         % Similarly, the cos() method could be cos2() to be more agresssive
%         % (expotential?) in the trail-off
%         z_acc = z_acc*cos(theta_r);
%     end

%     % Calculate cos(theta) difference of z basis to acceleration vector
    thrust_scale = dot(R_p(:,3),R_sp_c(:,3));
    
     if (z_acc <= 0) || (thrust_scale <= 0)
%        warning('Large thrust difference or reverse thrust detected')
        thrust_scale = 0;
     end
%     else
%         % Other options would be to set the value to zero over a set
%         % cut-off (e.g. pi/8) which would produce a stepping effect.
%         % Similarly, the cos() method could be cos2() to be more agresssive
%         % (expotential?) in the trail-off
%         z_acc = thrust_scale*z_acc;
%      end
     
%     vd_b = [0; 0; z_acc];
%     xid_b = [wd_b;
%              vd_b];

%     disp('Body vel')
    v_b = R_sp_s'*vel;
%     disp('Body acc')
%     R_sp_s'*acc;
%     disp('Body acc full')
%     vd_b = R_sp_s'*acc - vee_up(x(STATE_WXYZ_B,i-1))*R_p*R_sp_s'*vel;
%     vd_b = R_sp_s'*acc - vee_up(w_sp)*v_b;
%     vd_b = R_sp_s'*acc - vee_up(x(STATE_WXYZ_B,i-1))*x(STATE_VXYZ_B,i-1);
%      v_l = R_sp_s'*c(CONTROL_A,i) - vee_up(w_sp)*vel;
%     xi_b = [w_sp;
%             v_b];
%          
% 
%     vd_b = [0;0;thrust_scale*z_acc] - vee_up(w_sp)*v_b; 
%     a_b = thrust_scale*R_sp_s'*acc;
    vd_b = [0;0;thrust_scale*z_acc] - vee_up(x(STATE_WXYZ_B,i-1))*x(STATE_VXYZ_B,i-1);
    
    xid_b = [wd_b;
             vd_b];

%     eR = R'*R_sp;
%     ewd = eR*wd_sp - vee_up(w)*eR*w_sp;
%     eR*wd_sp
%     
%     tau = computed_torque_control(mass_c,[w_sp;v_b],xid_b);
    tau = computed_torque_control(mass_c,x(STATE_REDUCED,i-1),xid_b);

%     tau = Dq_c*xid_b + Cqqd_c*x(STATE_REDUCED,i-1)
%     tau(4:5) = 0;
%     tau = Dq_c*xid_b + Cqqd_c*[eR*w_sp;0;0;0];
    
%     tau = Dq_c*[wd_b;0;0;z_acc]
    % Simulte time step
    dx = spline_3d_sim_run(x(STATE_REDUCED,i-1), tau, mass_d);
    
%     % Construct 3D Affine to represent transformation matrix
%     gb_p = diag(ones(4,1));
%     gb_p(1:3,1:3) = R_p;
%     gb_p(1:3,4) = x(STATE_XYZ,i-1);
%         
%     % Propogate velocities from previous time step
%     eta_hat = zeros(4);
%     eta_hat(1:3,1:3) = vee_up(x(STATE_WXYZ_B,i-1));
%     eta_hat(1:3,4) = x(STATE_VXYZ_B,i-1);
%     
%     dgb = gb_p*eta_hat;
%     
%     % Update state vector for this time step
%     dR = dgb(1:3,1:3)*dt;
%     R = R_p + dR; %Current rotation for this time step
%     R_p_R = eye(3) + dR; % Rotation from R_p to R
%     x(STATE_R,i) = reshape(R,9,1);
%     x(STATE_XYZ,i) = x(STATE_XYZ,i-1) + dgb(1:3,4)*dt;
%     x(STATE_WXYZ_B,i) = x(STATE_WXYZ_B,i-1) + dx(STATE_REDUCED_W_B)*dt;
%     x(STATE_VXYZ_B,i) = R_p_R*x(STATE_VXYZ_B,i-1) + (dx(STATE_REDUCED_V_B) + R_p'*g_vec)*dt; %XXX: Rotates linear velocities to body frame at last time step and includes gravity acceleration
%     x(STATE_VXYZ,i) = R*x(STATE_VXYZ_B,i);%XXX: Update last as it is just a place-holder to save calculations later, but must be done after STATE_VXYZ_B

    % Update Dynamics
    % Angular Velocity
    x(STATE_WXYZ_B,i) = x(STATE_WXYZ_B,i-1) + dx(STATE_REDUCED_W_B)*dt;
%     x(STATE_WXYZ_B,i) = R_p'*R_sp_s*c(CONTROL_WXYZ_B,i); %XXX: Need to pull into the current frame of reference

    % Rotation
%     qd = 0.5*quatmultiply(x(STATE_Q,i-1)',[0,x(STATE_WXYZ_B,i)']);
%     q = quatnormalize(x(STATE_Q,i-1)' + qd*dt);
%     x(STATE_Q,i) = q';
    
    R = orthagonalize_rotm(R_p + R_p*vee_up(x(STATE_WXYZ_B,i-1)*dt));
    x(STATE_Q,i) = rotm2quat(R)';

    % Build up rotation matricies for body-world changes
%     R = quat2rotm(q);
%     R_p = quat2rotm(x(STATE_Q,i-1)');
    
    % Linear Velocity
%     x(STATE_VXYZ,i) = x(STATE_VXYZ,i-1) + (g_vec + R*dx(STATE_REDUCED_V_B))*dt;
%     x(STATE_VXYZ_B,i) = R'*x(STATE_VXYZ,i);

    x(STATE_VXYZ_B,i) = x(STATE_VXYZ_B,i-1) + (R'*g_vec + dx(STATE_REDUCED_V_B))*dt;
    x(STATE_VXYZ,i) = R*x(STATE_VXYZ_B,i);
    
    % Linear Position
    x(STATE_XYZ,i) = x(STATE_XYZ,i-1) + x(STATE_VXYZ,i)*dt;
end

disp(' ')


%% Plotting
disp('Plotting...')

name_order = '';
name_traj = '';

if order == 3
    name_order = 'Cubic';
elseif order == 5
    name_order = 'Quintic';
elseif order == 7
    name_order = 'Septic';
elseif order == 9
    name_order = 'Nonic';
else
    error(['Unknown plot title for order (', num2str(order)])
end

if strcmp(tname, 'yaw_only')
    name_traj = '$\mathbf{\psi}$-Only';
elseif strcmp(tname, 'x_only')
    name_traj = 'X-Only';
elseif strcmp(tname, 'y_only')
    name_traj = 'Y-Only';
elseif strcmp(tname, 'z_only')
    name_traj = 'Z-Only';
elseif strcmp(tname, 'circle_flat')
    name_traj = 'XY Circle';
elseif strcmp(tname, 'circle_flat_yaw')
    name_traj = 'XY$\mathbf{\psi}$ Circle';
elseif strcmp(tname, 'circle_raised')
    name_traj = 'XYZ Corkscrew';
elseif strcmp(tname, 'circle_raised_yaw')
    name_traj = 'XYZ$\mathbf{\psi}$ Corkscrew';
else
    error(['Unknown plot title for tname (', tname])
end

plot_title_3d = ['\textbf{', name_traj, ' ', name_order, ' Spline (3D)}'];
plot_title_x =  ['\textbf{', name_traj, ' ', name_order, ' Spline (X Axis)}'];
plot_title_y =  ['\textbf{', name_traj, ' ', name_order, ' Spline (Y Axis)}'];
plot_title_z =  ['\textbf{', name_traj, ' ', name_order, ' Spline (Z Axis)}'];
plot_title_psi =  ['\textbf{', name_traj, ' ', name_order, ' Spline ($\mathbf{\psi}$ Axis)}'];

ref_traj_x = cos(linspace(0,2*pi,num_traj_points*via_steps + 1));
ref_traj_y = sin(linspace(0,2*pi,num_traj_points*via_steps + 1));
ref_traj_z = linspace(0,5,num_traj_points*via_steps + 1);

ref_traj_dir = zeros(3,length(vias_psi(SPLINE_POS,:)));
yawR = eul2rotm([vias_psi(SPLINE_POS,:);
                 zeros(size(vias_psi(SPLINE_POS,:)));
                 zeros(size(vias_psi(SPLINE_POS,:)))]');
for ir = 1:length(vias_psi(SPLINE_POS,:))
    ref_traj_dir(:,ir) = yawR(:,:,ir)*[1;0;0];
end 

eul = quat2eul(x(STATE_Q,:)', 'ZYX');
roll = eul(:,3);
pitch = eul(:,2);
yaw = eul(:,1);

% eul_c = rotm2eul(reshape(c(CONTROL_R,:),3,3,length(t)), 'ZYX');
eul_c = quat2eul(c(CONTROL_Q,:)', 'ZYX');
roll_c = eul_c(:,3);
pitch_c = eul_c(:,2);
yaw_c = eul_c(:,1);

pos_error = [s_x(1,:);s_y(1,:);s_z(1,:)] - x(STATE_XYZ,:);
pos_RMSE = sqrt(mean((pos_error).^2,2));

w_error = c(CONTROL_WXYZ_B,:) - x(STATE_WXYZ_B,:);
w_RMSE = sqrt(mean((w_error).^2,2));

disp('Max Position Errors: ')
disp(max(abs(pos_error),[],2));
disp('Position RMSE: ')
disp(pos_RMSE)
disp('Omega RMSE: ')
disp(w_RMSE)

% error('Done')

%%
f1 = figure('Renderer','opengl');
    clf;
    
    title(plot_title_3d)
    hold on;
%     plot3(ref_traj_x, ref_traj_y, ref_traj_z, 'k--')
    plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
    plot3(x(STATE_X,:), x(STATE_Y,:), x(STATE_Z,:), 'b-');
    scatter3(vias_x(SPLINE_POS,:), vias_y(SPLINE_POS,:), vias_z(SPLINE_POS,:), 'ro')
    quiver3(vpx, vpy, vpz, ref_traj_dir(1,:), ref_traj_dir(2,:), ref_traj_dir(3,:), 'r')
    hold off;
    
    grid on;
    xlabel('X Position ($m$)');
    ylabel('Y Position ($m$)');
    zlabel('Z Position ($m$)');
    axis('equal')
%%

% error('Done')

f2 = figure('Renderer','opengl');
    clf;

    set(f2,'defaultAxesColorOrder',[0,0,0;0,0,0]);

    subplot(5,1,1)
        title(plot_title_x)

        hold on;
        yyaxis left;
        plot(t,s_x(1,:), 'r-');
        scatter(t_vias, vias_x(SPLINE_POS,:), 'ro')
        plot(t,x(STATE_X,:), 'b-');
        yyaxis right;
        plot(t,ref_traj_x, 'k--')
        hold off;

        grid on;
        yyaxis left;
        ylabel('Position ($m$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Sine Referece ($m$)');
        ylim([-maxlim maxlim]);
    subplot(5,1,2)
        hold on;
        plot(t,s_x(2,:), 'r-');
        scatter(t_vias, vias_x(SPLINE_VEL,:), 'ro')
        plot(t,x(STATE_VX,:), 'b-');
        hold off;

        grid on;
        ylabel('Velocity ($m/s$)');
    subplot(5,1,3)
        hold on;
        yyaxis left;
        plot(t,pitch_c, 'r-');
        plot(t,pitch, 'b-');
%         plot(t,atan2(s_x(3,:), g), 'g--');
        yyaxis right;
        scatter(t_vias, vias_x(SPLINE_ACC,:), 'ko')
        plot(t,s_x(3,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
%         xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch (rad)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Acceleration ($m/s^2$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        
    subplot(5,1,4)
        hold on;
        yyaxis left;
        plot(t,c(CONTROL_WY_B,:), 'r-');
        plot(t,x(STATE_WY_B,:), 'b-');
        yyaxis right;
        plot(t,s_x(4,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
%         xlabel('Time (s)');
        yyaxis left;
        ylabel('$\omega_{y}$ ($rad/s$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Jerk ($m/s^3$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);

    subplot(5,1,5)
        hold on;
        yyaxis left;
        plot(t,c(CONTROL_DWY_B,:), 'r-');
        yyaxis right;
        plot(t,s_x(5,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('$\dot{\omega}_{y}$ ($rad/s^{2}$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Snap ($m/s^4$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
%%
f3 = figure('Renderer','opengl');
    clf;

    set(f3,'defaultAxesColorOrder',[0,0,0;0,0,0]);

    subplot(5,1,1)
        title(plot_title_y)

        hold on;
        yyaxis left;
        plot(t,s_y(1,:), 'r-');
        scatter(t_vias, vias_y(SPLINE_POS,:), 'ro')
        plot(t,x(STATE_Y,:), 'b-');
        yyaxis right;
        plot(t,ref_traj_y, 'k--')
        hold off;

        grid on;
        yyaxis left;
        ylabel('Position ($m$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Cosine Referece ($m$)');
        ylim([-maxlim maxlim]);
    subplot(5,1,2)
        hold on;
        plot(t,s_y(2,:), 'r-');
        scatter(t_vias, vias_y(SPLINE_VEL,:), 'ro')
        plot(t,x(STATE_VY,:), 'b-');
        hold off;

        grid on;
        ylabel('Velocity ($m/s$)');
        
    subplot(5,1,3)
        hold on;
        yyaxis left;
        plot(t,roll_c, 'r-');
        plot(t,roll, 'b-');
%         plot(t,atan2(-s_y(3,:), g), 'g--');
        yyaxis right;
        scatter(t_vias, vias_y(SPLINE_ACC,:), 'ko')
        plot(t,s_y(3,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
%         xlabel('Time ($s$)');
        yyaxis left;
        ylabel('Roll ($rad$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Acceleration ($m/s^2$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        
    subplot(5,1,4)
        hold on;
        yyaxis left;
        plot(t,c(CONTROL_WX_B,:), 'r-');
        plot(t,x(STATE_WX_B,:), 'b-');
        yyaxis right;
        plot(t,s_y(4,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
%         xlabel('Time ($s$)');
        yyaxis left;
        ylabel('$\omega_{x}$ ($rad/s$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Jerk ($m/s^3$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);

    subplot(5,1,5)
        hold on;
        yyaxis left;
        plot(t,c(CONTROL_DWX_B,:), 'r-');
        yyaxis right;
        plot(t,s_y(5,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time ($s$)');
        yyaxis left;
        ylabel('$\dot{\omega}_{x}$ ($rad/s^2$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Snap ($m/s^4$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
%%   
f4 = figure('Renderer','opengl');
    clf;

    set(f4,'defaultAxesColorOrder',[0,0,0;0,0,0]);

    subplot(5,1,1)
        title(plot_title_z)

        hold on;
        plot(t,s_z(1,:), 'r-');
        scatter(t_vias, vias_z(SPLINE_POS,:), 'ro')
        plot(t,x(STATE_Z,:), 'b-');
        hold off;

        grid on;
        ylabel('Position ($m$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        
    subplot(5,1,2)
        hold on;
        plot(t,s_z(2,:), 'r-');
        scatter(t_vias, vias_z(SPLINE_VEL,:), 'ro')
        plot(t,x(STATE_VZ,:), 'b-');
        hold off;

        grid on;
        ylabel('Velocity ($m/s$)');
        
    subplot(5,1,3)
        hold on;
        plot(t,s_z(3,:), 'r-');
        scatter(t_vias, vias_z(SPLINE_ACC,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .2f');
%         xlabel('Time (s)');
        ylabel('Acceleration ($m/s^2$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        
    subplot(5,1,4)
        title(plot_title_psi)
        
        hold on;
        plot(t,yaw_c, 'r-');
        scatter(t_vias, wrapToPi(vias_psi(SPLINE_POS,:)), 'ro')
        plot(t,yaw, 'b-');
        hold off;

        grid on;
        ytickformat('% .2f');
%         xlabel('Time ($s$)');
        ylabel('$\psi$ ($rad$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        
    subplot(5,1,5)
        hold on;
        plot(t,c(CONTROL_WZ_B,:), 'r-');
        scatter(t_vias, vias_psi(SPLINE_VEL,:), 'ro')
        plot(t,x(STATE_WZ_B,:), 'b-');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time ($s$)');
        ylabel('$\dot{\psi}$ ($rad/s$)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        
% print(f1, plot_titles{2}, '-depsc')
%%
% 
% frame_size = 0.45;
% prop_rad = 0.075;
% 
% f5 = figure('Renderer','opengl');
%     ax = axes();
%     
%     axis([-2,2,-2,2,-2,2]);
%     axis square;
%     grid on;
%     
% for i = 1:(1/dt)/20:length(t)
%     tic;
%     
%     cla;
%     draw_quad( ax, x(:,i), frame_size, prop_rad );
%     hold on;
%     plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
%     plot3(x(STATE_X,1:i), x(STATE_Y,1:i), x(STATE_Z,1:i), 'b-');
%     hold off;
%     drawnow;
%     
%     t_now = toc;
%     pause((1/20) - t_now)
% end
% 
% disp('Done!')

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    