%% Setup

close all;
clear;
clc;
%%
set(0,'defaultTextInterpreter','latex');

% Enables plots if >0
show_plots = 0;
% Enables animation if >0
% Also acts as a speed multiplier, 0.5 will be half animation speed
show_animation = 0.0; %1.0;
save_animation = 0;
% Enables keyframe view of animation
% Also acts as the keyframe interval, 1.0 will be a keyframe every second
show_keyframes =0.0; %0.2;
keyframe_tf = 1.7; % If greater than 0, will stop showing keyframes at kf_tf

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

% 'hover'
% 'x_only'
% 'y_only'
% 'z_only'
% 'yaw_only'
% 'circle_flat'
% 'circle_flat_yaw'
% 'circle_raised'
% 'circle_raised_yaw'
tname = 'hover';

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

% https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2017/RD_HS2017script.pdf
% Pg. 76
% Hence, the eigenfrequency and a dimensionless damping value of the system
% are given by:
% ω=√kp,(3.78)
% D=kd/2√kp.(3.79)
% Critical damping is achieved for D=1, overcritical damping for D>1 and
% un-dercritical  damping  for D <1. The  compliance  of  the  controller
% can be adjusted by varying kp.  For example, assuming that the time
% constant respectively oscillation frequency around the nominal point
% should be 3 Hz, the ideal control gain for kp is 350. Furthermore, 
% critical damping requires kd= 37. This holds as good starting values for
% controller gain tuning.

%XXX: This arrangement of gain frequencies gives a response of a critically
%damped system for both position and attitude tracking. w0r is set to be an
%order of magnitude faster in response than w0p, which seems to give good
%results (but I'm not 100% sure why, frequency responses mixing? Maybe a 
% good reference would be similar to a cascade controller timings - should
% be 5-20x more on who you ask)
w0p = 2;
w0r = 10*w0p;
%A bad result could be obtained with the following parameters:
% No control:
% w0p = 0;
% w0r = 0;
% Early Inversion:
% w0p = 3.2;
% w0r = 12;
% Critical Inversion:
% w0p = 3.5;
% w0r = 12;
% Converging Spiral:
% w0p = 4;
% w0r = 12;
% Diverging Spiral:
% w0p = 6;
% w0r = 12;



% KxP = 2.0;      % Position tracking P gain
% KxdP =  6.0;    % Velocity tracking P gain
% KtP = 60.0;     % Angular tracking P gain
% KtdP = 10.0;    % Angular Rate tracking P gain
KxP = w0p^2;      % Position tracking P gain
KxdP =  2*w0p;    % Velocity tracking P gain
KtP = w0r^2;     % Angular tracking P gain
KtdP = 2*w0r;    % Angular Rate tracking P gain

yaw_w = 0.6;    % Yaw weighting for rotational tracking
theta_max = deg2rad(30); % Maximum thrust vectoring angle (from vertical)


%% Script Variables
disp('Generating...')

via_steps = ((tf - t0) / dt) / (num_via_points - 1);
num_traj_points = num_via_points - 1;
[vpx, vpy, vpz, vppsi] = trajectory_vias(tname, num_via_points);

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
CONTROL_DV_B = 17;  % Body acceleration vector length

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

% Initial state
x0 = [eul2quat([vppsi(1),0,0])';    % Rotation (World)
      vpx(1); vpy(1); vpz(1);       % Position (World)
      zeros(3,1);                   % Linear Velocity (World)
      zeros(3,1);                   % Angular Velocity (Body)
      zeros(3,1)];                  % Linear Velocity (Body)

% Bad starting states:
% x0(STATE_Q) = eul2quat([pi/2,0,0])';    % Full yaw error rotation (World)
x0(STATE_Q) = eul2quat([0,0,deg2rad(179)])';    % (Almost) Full roll error rotation (World)
% x0(STATE_XYZ) = [-1;0;1];    % Full pitch error rotation (World)



g = 9.80665;
g_vec = [0;0;-g];

t = t0:dt:tf;
t_vias = linspace(t0,tf,num_via_points);

if logical(mod(t_vias,dt))
    error('t_vias entries must be a multiple of dt')
end

vias_x = gen_vias(via_est_method, vpx, t_vias);
vias_y = gen_vias(via_est_method, vpy, t_vias);
vias_z = gen_vias(via_est_method, vpz, t_vias);
vias_psi = gen_vias(via_est_method, vppsi, t_vias);
[st, s_x] = gen_spline(t_vias, vias_x, order, dt);
[~, s_y] = gen_spline(t_vias, vias_y, order, dt);
[~, s_z] = gen_spline(t_vias, vias_z, order, dt);
[~, s_psi] = gen_spline(t_vias, vias_psi, order, dt);

% Sanity check:
% Ensure that our time vectors ended up the same size, and same dt,
% probably fine if these two cases line up
timescale_checks = logical([(size(st) ~= size(t)), ...
                            ((st(2)-st(1)) ~= (t(2)-t(1)))]);
if timescale_checks
    error('Error constructing spline times')
end

% Build state vector
x = zeros(size(x0,1),length(t));
x(:,1) = x0;

% Build control vector
c = zeros(17,length(t));


%% Simulation
reverseStr = '';
progressLast = 0;

% Control error integrator
px4_eang_int = zeros(3,1);
px4_rang_int = zeros(3,1);

for i = 2:length(t)
    %% Status Messages
    progress = 100*i/length(t);
    if progress > progressLast + 0.1
        msg = sprintf('Simulating... %3.1f', progress);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        progressLast = progress;
    end

    %% Controller
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

    % Calculate angle axis of acceleration vector to vertical
%     avec_a_a = vrrotvec([0;0;1],c(CONTROL_A,i));
%     theta_a = avec_a_a(4);
%     if abs(theta_a) > theta_max
%        warning('Large thrust vector detected')
%     end

    % Calculate the mapping for body-rate setpoints
%     [R_sp_s, q_sp_s] = rot_from_vec_yaw(acc,yaw);
    [R_sp_s,w_sp,wd_sp] = map_angles_rates_accels(acc, jerk, snap, yaw, dyaw, ddyaw);
    c(CONTROL_WXYZ_B,i) = w_sp;
    c(CONTROL_DWXYZ_B,i) = wd_sp;

    [eR_s, ew, ewd] = error_R_w_wd_trans(R_sp_s, R_p, ...
                                         c(CONTROL_WXYZ_B,i), x(STATE_WXYZ_B,i-1), ...
                                         c(CONTROL_DWXYZ_B,i));

    % Calculate a different attitude rotation error function for the
    % that applies the control for the position/velocity terms.
%     c(CONTROL_R_E,i) = error_q_att(q_sp_c, x(STATE_Q,i-1)', yaw_w);
    c(CONTROL_R_E,i) = 0.5*vee_down(R_p'*R_sp_c - R_sp_c'*R_p); % XXX: This won't work at inversion singularity (i.e R_p = -R_sp_c)

    % SO(3) PD Tracking
    wd_b = ewd ...
         + KtdP*ew ...
         + KtP*c(CONTROL_R_E,i);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % PX4 PID Tracking
%     MC_ANG_P = 6.5;
%     MC_RATE_P = 7.5;
%     MC_RATE_I = 2.5;
%     MC_RATE_D = 0.15;
%     KP_t = MC_ANG_P*MC_RATE_P;
%     KI_t = MC_RATE_I*MC_ANG_P;
%     KD_t = MC_RATE_P;
%     KI = MC_RATE_I;
%     KD = MC_RATE_D;
%
%     c(CONTROL_R_E,i) = error_q_att(q_sp_c, x(STATE_Q,i-1)', yaw_w);
%     c(CONTROL_WXYZ_B,i) = zeros(3,1);
%     ew = c(CONTROL_WXYZ_B,i) - x(STATE_WXYZ_B,i-1);
%     ewd = zeros(3,1);
%     wd_sp = zeros(3,1);
%     if i > 2
%         ewd = wd_sp - (x(STATE_WXYZ_B,i-1) - x(STATE_WXYZ_B,i-2))/dt;
%     end
%     px4_eang_int = px4_eang_int + c(CONTROL_R_E,i)*dt;
%     px4_rang_int = px4_rang_int + ew*dt;
%
%     % Control Structure resmbles PtIt-PwIw-Dwd
%     wd_b = KP_t*c(CONTROL_R_E,i) + KD_t*ew + KI_t*px4_eang_int + KI*px4_rang_int + KD*ewd;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Actual PX4 PID Loop structure
%     w_sp = MC_ANG_P*error_q_att(q_sp_c, x(STATE_Q,i-1)', yaw_w);
%     ew = w_sp - x(STATE_WXYZ_B,i-1);
%     wd_sp = zeros(3,1);
%     ewd = zeros(3,1);
%     if i > 2
%         ewd = wd_sp - (x(STATE_WXYZ_B,i-1) - x(STATE_WXYZ_B,i-2))/dt;
%     end
%
%     px4_rang_int = px4_rang_int + ew*dt;
%
%     wd_b = MC_RATE_P*ew + MC_RATE_I*px4_rang_int + MC_RATE_D*ewd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Thrust Vectoring
%     z_acc = norm(acc);
    z_acc = norm(c(CONTROL_A,i));

    % Calculate cos(theta) difference of z basis to acceleration vector
    %
    % Other options would be to set the value to zero over a set
    % cut-off (e.g. pi/8) which would produce a stepping effect.
    % Similarly, the cos() method could be cos2() to be more agresssive
    % (expotential?) in the trail-off
    thrust_scale = dot(R_sp_c(:,3),R_p(:,3));

    % Inversion protection
    if (z_acc <= 0) || (thrust_scale <= 0)% || (sign_no_zero(R_p(3,3)) ~= sign_no_zero(R_sp_c(3,3)))
%        warning('Large thrust difference or reverse thrust detected')
        thrust_scale = 0;
    end


%     vd_b = [0; 0; z_acc];
%     xid_b = [wd_b;
%              vd_b];

%     disp('Body vel')
%     v_b = R_sp_s'*vel;
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
    c(CONTROL_DV_B,i) = thrust_scale*z_acc;
    vd_b = [0;0;c(CONTROL_DV_B,i)];% - vee_up(x(STATE_WXYZ_B,i-1))*x(STATE_VXYZ_B,i-1);

%     disp('state:')
%     vee_up(x(STATE_WXYZ_B,i-1))*x(STATE_VXYZ_B,i-1)
%     disp('ref:')
%     % vee_up(w_sp)*v_b ==> 0
%     vee_up(w_sp)*v_b ==> 0

    xid_b = [wd_b;
             vd_b];

%     eR = R'*R_sp;
%     ewd = eR*wd_sp - vee_up(w)*eR*w_sp;
%     eR*wd_sp
%
%     tau = computed_torque_control(mass_c,[w_sp;v_b],xid_b);
    tau = control_feedback_linearisation_3d(mass_c,x(STATE_REDUCED,i-1),xid_b);

    %% Simulte Time Step
    dx = spline_3d_sim_run(x(STATE_REDUCED,i-1), tau, mass_d);

    % Update Dynamics
    % Angular Velocity
    x(STATE_WXYZ_B,i) = x(STATE_WXYZ_B,i-1) + dx(STATE_REDUCED_W_B)*dt;
%     % Perfect angular velocity tracking
%     x(STATE_WXYZ_B,i) = R_p'*R_sp_s*c(CONTROL_WXYZ_B,i); %XXX: Need to pull into the current frame of reference

    % Rotation
    R = orthagonalize_rotm(R_p + R_p*vee_up(x(STATE_WXYZ_B,i-1)*dt));
    x(STATE_Q,i) = rotm2quat(R)';

    % Linear Velocity
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

if strcmp(tname, 'hover')
    name_traj = 'Hover';
elseif strcmp(tname, 'yaw_only')
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

R_state = quat2rotm(x(STATE_Q,:)');
tv_R_state = squeeze(R_state(:,3,:));
tv_spline = [s_x(SPLINE_ACC,:);s_y(SPLINE_ACC,:);s_z(SPLINE_ACC,:)] +  - g_vec;
tv_S_state = tv_spline./vecnorm(tv_spline,2,1);
tv_error = thrustvec_angle_error( tv_S_state, tv_R_state);

disp('Max Position Errors: ')
disp(max(abs(pos_error),[],2));
disp('Position RMSE: ')
disp(pos_RMSE)
disp('Omega RMSE: ')
disp(w_RMSE)

if show_plots > 0
    %%
    f1 = figure('Renderer','opengl');
        clf;

        title(plot_title_3d)
        hold on;
    %     plot3(ref_traj_x, ref_traj_y, ref_traj_z, 'k--')
        plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
        plot3(x(STATE_X,:), x(STATE_Y,:), x(STATE_Z,:), 'b-');
        scatter3(vias_x(SPLINE_POS,:), vias_y(SPLINE_POS,:), vias_z(SPLINE_POS,:), 'ro')
        scatter3(x0(STATE_X), x0(STATE_Y), x0(STATE_Z), 'bx')
        quiver3(vpx, vpy, vpz, ref_traj_dir(1,:), ref_traj_dir(2,:), ref_traj_dir(3,:), 'r')
        hold off;
        axis('equal')
        maxlim = max([abs(ylim),abs(xlim)]);
        xlim([-maxlim maxlim]);
        ylim([-maxlim maxlim]);
        view(55,35);

        grid on;
        xlabel('X Position ($m$)');
        ylabel('Y Position ($m$)');
        zlabel('Z Position ($m$)');
        %axis('equal')
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
            
% print(f1, ['./figures/',plot_titles{2}], '-depsc')
end

%%
if show_animation > 0
    flight_space = 3.5; %3.5; 6.0; 9.0;
    frame_size = 0.45;
    prop_rad = 0.075;
    
    framerate = 30; %Hz
    sim_rate = framerate/show_animation; %Hz
    % sim_tstep needs to be a positive integer, might not be if dt is not a
    % multiple of sim_rate
    sim_tstep = round(1/(dt*sim_rate));
    
    f5 = figure('Renderer','opengl');
    ax = axes();

    axis([-flight_space,flight_space,-flight_space,flight_space,-flight_space,flight_space]);
    axis square;
    grid on;
%     ax.Units = 'pixels';
%     axpos = ax.Position;
%     axti = ax.TightInset;
%     axrect = [-axti(1), -axti(2), axpos(3)+axti(1)+axti(3), axpos(4)+axti(2)+axti(4)];

    vid_write = VideoWriter('./figures/animation', 'Archival');
%     vid_write.Quality = 95;
    vid_write.FrameRate = framerate;
    if save_animation > 0
        open(vid_write);
    end
    
    for i = 1:sim_tstep:length(t)
        tic;

        cla;
        draw_quad( ax, x(:,i), frame_size, prop_rad );
        hold on;
        plot3(ax, s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
        plot3(ax, x(STATE_X,1:i), x(STATE_Y,1:i), x(STATE_Z,1:i), 'b-');
        hold off;
        
        view(55,35);

        drawnow;
        
        if save_animation > 0
%             writeVideo(vid_write,getframe(ax,axrect))
            writeVideo(vid_write,getframe(ax));
        else
            t_now = toc;
            pause((1/20) - t_now);
        end
    
    end

    if save_animation > 0
        close(vid_write);
        disp(['Animation writen to: ', vid_write.Path])
        disp('Convert animation to MP4 with: ffmpeg -i animation.mj2 -q:v 0 animation.mp4')
    end
    
    disp('Finished animation!')
end

%%
if show_keyframes > 0
    flight_space = 2.2; %3.5; 6.0; 9.0;
    frame_size = 0.45;
    prop_rad = 0.075;
    
    still_scaler = 0.2;
    
    
    still_frame_size = still_scaler*frame_size;
    still_prop_rad = still_scaler*prop_rad;
    
    kf_tf = tf;
    if keyframe_tf > 0
        kf_tf = keyframe_tf;
    end
    kf_tf_i = find(t <= kf_tf,1,'last');
    
    kf_tstep = round((1/dt)*show_keyframes);
    
    f6 = figure('Renderer','opengl');
    
%     ax1 = subplot(2,1,1);
    ax1 = axes();

        cla;
        plot3(ax1, s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
        hold on;
        plot3(ax1, x(STATE_X,:), x(STATE_Y,:), x(STATE_Z,:), 'b-');

        for i = 1:kf_tstep:kf_tf_i
            draw_quad( ax1, x(:,i), frame_size, prop_rad );

        end
        hold off;

        axis([-flight_space,flight_space,-flight_space,flight_space,-flight_space,flight_space]);
        axis square;
        grid on;
        xlabel('X Position ($m$)');
        ylabel('Y Position ($m$)');
        zlabel('Z Position ($m$)');
        view(55,35);
        
    f7 = figure('Renderer','opengl');  
%     ax2 = subplot(2,1,2);
    ax2 = axes();

        cla;
        hold on;
        for i = 1:kf_tstep:kf_tf_i   
            xstill = x(:,i);
            xstill(STATE_X) = 0;
            xstill(STATE_Y) = t(i);
            xstill(STATE_Z) = 0;
            draw_quad( ax2, xstill, still_frame_size, still_prop_rad );

        end
        hold off;

%         axis([-flight_space,flight_space,-flight_space,flight_space,-flight_space,flight_space]);
        axis tight;
        axis equal;
        grid on;
        xticks([]);
        ylabel('Time ($s$)');
        zticks([]);
        view(55,35);
        
%     print(f6, ['./figures/keyframes_3d_',name_traj], '-dpng')
%     print(f7, ['./figures/keyframes_time_',name_traj], '-dpng')
end









