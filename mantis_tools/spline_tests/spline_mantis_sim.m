% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

%% Setup

error('This is an older file, use mantis_sim.m instead.')

close all;
clear;
clc;

addpath( genpath('./spatial_v2') );
set(0,'defaultTextInterpreter','latex');

% Print results in latex tablular format if >0 (normally printed otherwise)
print_latex_results = 1; 
% Enables plots if >0
show_plots = 1;
% Enables animation if >0
% Also acts as a speed multiplier, 0.5 will be half animation speed
show_animation = 0.0; %1.0;
save_animation = 0;
% Enables keyframe view of animation
% Also acts as the keyframe interval, 1.0 will be a keyframe every second
show_keyframes =0.0; %0.2;
keyframe_tf = 1.7; % If greater than 0, will stop showing keyframes at kf_tf
% Enable the full 3D animation from the spatial_v2 library if >0
% Camera setup for this animation as well
show_full_animation = 0;
camera.body = 0; % Set to 6 to track the robot base
camera.direction = [-3 -3 2];	% Downwards better viewing angle
camera.zoom = 0.2;


%% User Variables

    % Timings
    config.t0 = t0;
    config.dt = 1/1000; % Simulation rate
    config.cdt = 1/250; % Controller rate
    config.tf = 10;

    % Model Properties
    % Frame types:
    % 'quad'
    % 'hex'
    % Manipulator types:
    % 'serial'
    % Number of maniplator links/joints: n
    frame_type = 'quad_x4';
    manip_type = 'serial';
    n = 2;

    % Spline Order:
    %   cubic:  3
    %   quntic: 5
    %   septic: 7
    %   nonic:  9
    order = 9;

    % Number of vias to use during generation
    num_via_points = 9;

    % Trajectory Name:
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

    % Joint Trajectory Names:
    % (Must be a cell array of strings of size n)
    % 'steady_0' - Joint angles 0
    % 'steady_90' - Joint angles 0
    % 'swing_part' - Joint angles 0->pi/4
    % 'swing_half' - Joint angles 0->pi/2
    % 'swing_full' - Joint angles -pi/2->pi/2
    tname_r = {
        'steady_90';
        'steady_0'
    };

    % via_est_method = 'linear';
    via_est_method = 'fdcc';

    % Control Method:
    % 'npid_px4' - Nonlinear PID (Regulating, PX4 Structure)
    % 'npid_exp' - Nonlinear PID (Regulating, Expanded PX4 Structure)
    % 'npid'     - Nonlinear PID (Tracking)
    % 'ctc'      - Computed Torque Control (Tracking)
    % 'feed'     - Feed-Forward Compensation (Tracking)
    control_method = 'npid_px4';
    control_fully_actuated = 0; % Allows the platform to actuate in all directions if >0

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
    w0t = 10*w0p;
    w0r = 4;

    %A bad result could be obtained with the following parameters:
    % No control:
    % w0p = 0;
    % w0t = 0;
    % Early Inversion:
    % w0p = 3.2;
    % w0t = 12;
    % Critical Inversion:
    % w0p = 3.5;
    % w0t = 12;
    % Converging Spiral:
    % w0p = 4;
    % w0t = 12;
    % Diverging Spiral:
    % w0p = 6;
    % w0t = 12;

    KxP = w0p^2;    % Position tracking P gain
    KxD = 2*w0p;    % Velocity tracking P gain
    KtP = w0t^2;    % Angular tracking P gain
    KtD = 2*w0t;    % Angular rate tracking P gain
    KrP = w0r^2;    % Joint tracking P gain
    KrD = 2*w0r;    % Joint rate tracking P gain

    yaw_w = 0.6;    % Yaw weighting for rotational tracking
    theta_max = deg2rad(30); % Maximum thrust vectoring angle (from vertical)


%% Script Variables

% Create a lookup for all the state variables to make it easier to address
% all sorts of indexes in our arrays
sn = state_names_lookup(n);

model_d = gen_model(frame_type, manip_type, n, camera);
model_c = model_d; % Mass properties for controller

controller_rate = floor((1/dt)/(1/cdt));

via_steps = ((tf - t0) / dt) / (num_via_points - 1);
num_traj_points = num_via_points - 1;
[vpx, vpy, vpz, vppsi] = trajectory_vias(tname, num_via_points);
vpr = cell(n,1);
r0 = zeros(n,1);
for i = 1:n
    vpr{i} = trajectory_vias_r(tname_r{i}, num_via_points);
    r0(i) = vpr{i}(1); % Easier to capture initial joint states in this loop
end

% Initial state
x0 = [eul2quat([vppsi(1),0,0])';    % Rotation (World)
      vpx(1); vpy(1); vpz(1);       % Position (World)
      r0;                           % Joint Positions (Body)
      zeros(3,1);                   % Linear Velocity (World)
      zeros(3,1);                   % Angular Velocity (Body)
      zeros(3,1);                   % Linear Velocity (Body)
      zeros(n,1)];                  % Joint Velocity (Body)

% Bad starting states:
% x0(sn.STATE_Q) = eul2quat([pi/2,0,0])';    % Half yaw error rotation (World)
x0(sn.STATE_Q) = eul2quat([0,0,deg2rad(179)])';    % (Almost) Full roll error rotation (World)
% x0(sn.STATE_XYZ) = [1;1;0];    % Positional Error (World)
% x0(sn.STATE_R) = zeros(n,1);   % Arm down Joint Error (World)


% Display starting state
disp('--== Start State ==--');
disp(x0([sn.STATE_Q,sn.STATE_XYZ,sn.STATE_R])');

% Rest of simulator setup
disp('----------------');
disp(' ')
disp('Generating...')

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
vias_r = cell(n,1);
for i = 1:n
    vias_r{i} = gen_vias(via_est_method, vpr{i}, t_vias);
end
[st, s_x] = gen_spline(t_vias, vias_x, order, dt);
[~, s_y] = gen_spline(t_vias, vias_y, order, dt);
[~, s_z] = gen_spline(t_vias, vias_z, order, dt);
[~, s_psi] = gen_spline(t_vias, vias_psi, order, dt);
s_r = cell(n,1);
for i = 1:n
    [~, s_r{i}] = gen_spline(t_vias, vias_r{i}, order, dt);
end

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
c = zeros(20+3*n-1,length(t));


%% Simulation
reverseStr = '';
progressLast = 0;

control_tick = 0;
control_tau_last = zeros(length(sn.STATE_REDUCED),1);

% Controller integrator variables
npid_ang_integrator = zeros(3,1);
npid_omega_integrator = zeros(3,1);
manip_integrator = zeros(n,1);

for i = 2:length(t)
    %% Status Messages
    progress = 100*i/length(t);
    if progress > progressLast + 0.1
        msg = sprintf('Simulating... %3.1f', progress);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        progressLast = progress;
    end
    
    if control_tick <= 1
        %% Guidance
        % Spline reference vectors
        pos = [s_x(sn.SPLINE_POS,i);s_y(sn.SPLINE_POS,i);s_z(sn.SPLINE_POS,i)];
        vel = [s_x(sn.SPLINE_VEL,i);s_y(sn.SPLINE_VEL,i);s_z(sn.SPLINE_VEL,i)];
        acc = [s_x(sn.SPLINE_ACC,i);s_y(sn.SPLINE_ACC,i);s_z(sn.SPLINE_ACC,i)] - g_vec; %XXX: -g to have a positive term for gravity added
        jerk = [s_x(sn.SPLINE_JERK,i);s_y(sn.SPLINE_JERK,i);s_z(sn.SPLINE_JERK,i)];
        snap = [s_x(sn.SPLINE_SNAP,i);s_y(sn.SPLINE_SNAP,i);s_z(sn.SPLINE_SNAP,i)];

        yaw = s_psi(sn.SPLINE_POS,i);
        dyaw = s_psi(sn.SPLINE_VEL,i);
        ddyaw = s_psi(sn.SPLINE_ACC,i);
        
        r = zeros(n,1);
        rd = zeros(n,1);
        rdd = zeros(n,1);
        for j = 1:n
            r(j) = s_r{j}(sn.SPLINE_POS,i);
            rd(j) = s_r{j}(sn.SPLINE_VEL,i);
            rdd(j) = s_r{j}(sn.SPLINE_ACC,i);
        end
        
        % Projection to SO(3)/so(3)
        [R_sp_s, w_sp_s, wd_sp_s] = map_angles_rates_accels(acc, jerk, snap, yaw, dyaw, ddyaw);

        
        %% Control Law
        if strcmp(control_method, 'npid_px4')
            if i > 2
                x_p = x(:,i-2);
            else
                x_p = x(:,i-1);
            end
            
            [tau_b, acc_c, q_sp_c, npid_omega_integrator ] = control_nonlinear_pid_px4( model_c, ...          
                                                     pos, vel, acc, yaw, ...
                                                     x(:,i-1), x_p, ...
                                                     cdt, npid_omega_integrator);
                                                                                 
            [tau_r, manip_integrator] = control_manip_decoupled( model_c, ...
                                        r, rd, ...
                                        x(:,i-1), ...
                                        cdt, manip_integrator);
            
            tau_full = [tau_b;tau_r];
            
        elseif strcmp(control_method, 'npid_exp')
%             if i > 2
%                 x_p = x(:,i-2);
%             else
%                 x_p = x(:,i-1);
%             end
%             
%             [tau_full, acc_c, q_sp_c, npid_ang_integrator, npid_omega_integrator ] = control_nonlinear_pid_exp( model_c, ...          
%                                                      pos, vel, acc, yaw, ...
%                                                      x(:,i-1), x_p, ...
%                                                      cdt, npid_ang_integrator, npid_omega_integrator);
        elseif strcmp(control_method, 'npid')
%             % Instead of normalising the thrust vector, simply use F=ma
%             kT = mass_c.m;
%             [tau_full, acc_c, q_sp_c, npid_ang_integrator ] = control_nonlinear_pid( model_c, ...          
%                                                      pos, vel, acc, ...
%                                                      yaw, w_sp_s, ...
%                                                      x(:,i-1), kT, ...
%                                                      KxP, KxD, KtP, KtD, yaw_w, ...
%                                                      cdt, npid_ang_integrator);
        elseif strcmp(control_method, 'ctc')
            [tau_full, acc_c, q_sp_c ] = control_computed_torque( model_c, ...
                                         pos, vel, acc, ...
                                         yaw, R_sp_s, w_sp_s, wd_sp_s, ...
                                         r, rd, rdd, ...
                                         x(:,i-1), ...
                                         KxP, KxD, KtP, KtD, yaw_w, KrP, KrD);
        elseif strcmp(control_method, 'feed')
            if i > 2
                x_p = x(:,i-2);
            else
                x_p = x(:,i-1);
            end
            
            [tau_b, acc_c, q_sp_c, npid_omega_integrator ] = control_feed_forward( model_c, ...
                                         pos, vel, acc, yaw, ...
                                         x(:,i-1), x_p, ...
                                         cdt, npid_omega_integrator);
                                         
            [tau_r, manip_integrator] = control_manip_decoupled( model_c, ...
                                        r, rd, ...
                                        x(:,i-1), ...
                                        cdt, manip_integrator);
            tau_full = [tau_b;tau_r];
        else
            error('Could not determine control method to use')
        end
        
        % Handle fully/under actuated cases
        if control_fully_actuated > 0
            tau = tau_full;
        else
            tau = [tau_full(1:3);0;0;tau_full(6:end)];
        end
        
        
        %% Save control inputs
        c(sn.CONTROL_A,i) = acc_c;
        c(sn.CONTROL_Q,i) = q_sp_c';
        c(sn.CONTROL_WXYZ_B,i) = w_sp_s;
        c(sn.CONTROL_DWXYZ_B,i) = wd_sp_s;
        c(sn.CONTROL_R,i) = r;
        c(sn.CONTROL_RD,i) = rd;
        c(sn.CONTROL_RDD,i) = rdd;
        
        
        %% Control loop management
        % Save control signal for next step
        control_tau_last = tau;
        % Set the control loop run again shortly
        control_tick = controller_rate;
        
        
    else
        % Use the control signal from the last control tick
        tau = control_tau_last;
        c(sn.CONTROL_A,i) = c(sn.CONTROL_A,i-1);
        c(sn.CONTROL_Q,i) = c(sn.CONTROL_Q,i-1);
        c(sn.CONTROL_WXYZ_B,i) = c(sn.CONTROL_WXYZ_B,i-1);
        c(sn.CONTROL_DWXYZ_B,i) = c(sn.CONTROL_DWXYZ_B,i-1);
        c(sn.CONTROL_R,i) = c(sn.CONTROL_R,i-1);
        c(sn.CONTROL_RD,i) = c(sn.CONTROL_RD,i-1);
        c(sn.CONTROL_RDD,i) = c(sn.CONTROL_RDD,i-1);
        
        control_tick = control_tick - 1;
    end
    
    
    %% Simulte Time Step
    dx = spline_mantis_sim_run(x(:,i-1), tau, model_d);

    % Update Dynamics
    % Previous rotation matrix (for ease of use)
    R_p = quat2rotm(x(sn.STATE_Q,i-1)');
    
    % Joints
    x(sn.STATE_RD,i) = x(sn.STATE_RD,i-1) + dx(sn.STATE_REDUCED_RD)*dt;
    x(sn.STATE_R,i) = x(sn.STATE_R,i-1) + x(sn.STATE_RD,i-1)*dt;
    
    
    % Angular Velocity
    x(sn.STATE_WXYZ_B,i) = x(sn.STATE_WXYZ_B,i-1) + dx(sn.STATE_REDUCED_W_B)*dt;
%     % Perfect angular velocity tracking
%     x(sn.STATE_WXYZ_B,i) = R_p'*R_sp_s*c(sn.CONTROL_WXYZ_B,i); %XXX: Need to pull into the current frame of reference

    % Rotation
    R = orthagonalize_rotm(R_p + R_p*vee_up(x(sn.STATE_WXYZ_B,i-1)*dt));
    x(sn.STATE_Q,i) = rotm2quat(R)';

    % Linear Velocity
    % Propogate linear velocity in the world frame
    x(sn.STATE_VXYZ,i)= x(sn.STATE_VXYZ,i-1) + (g_vec + R_p*dx(sn.STATE_REDUCED_V_B))*dt;
    % Then convert back to the body frame for the current time step
    x(sn.STATE_VXYZ_B,i) = R'*x(sn.STATE_VXYZ,i);
    
    % Linear Position
    x(sn.STATE_XYZ,i) = x(sn.STATE_XYZ,i-1) + x(sn.STATE_VXYZ,i-1)*dt;
end

disp(' ') % Add some spacing, needed due to previous printf()
disp(' ')


%% Data Analysis

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
plot_title_r =  ['\textbf{', name_traj, ' ', name_order, ' Spline'];

ref_traj_x = cos(linspace(0,2*pi,num_traj_points*via_steps + 1));
ref_traj_y = sin(linspace(0,2*pi,num_traj_points*via_steps + 1));
ref_traj_z = linspace(0,5,num_traj_points*via_steps + 1);

ref_traj_dir = zeros(3,length(vias_psi(sn.SPLINE_POS,:)));
yawR = eul2rotm([vias_psi(sn.SPLINE_POS,:);
                 zeros(size(vias_psi(sn.SPLINE_POS,:)));
                 zeros(size(vias_psi(sn.SPLINE_POS,:)))]');
for ir = 1:length(vias_psi(sn.SPLINE_POS,:))
    ref_traj_dir(:,ir) = yawR(:,:,ir)*[1;0;0];
end

eul = quat2eul(x(sn.STATE_Q,:)', 'ZYX');
roll = eul(:,3);
pitch = eul(:,2);
yaw = eul(:,1);

% eul_c = rotm2eul(reshape(c(sn.CONTROL_R,:),3,3,length(t)), 'ZYX');
eul_c = quat2eul(c(sn.CONTROL_Q,:)', 'ZYX');
roll_c = eul_c(:,3);
pitch_c = eul_c(:,2);
yaw_c = eul_c(:,1);

pos_error = [s_x(1,:);s_y(1,:);s_z(1,:)] - x(sn.STATE_XYZ,:);
max_pos_error = max(abs(pos_error),[],2);
pos_RMSE = sqrt(mean((pos_error).^2,2));

w_error = c(sn.CONTROL_WXYZ_B,:) - x(sn.STATE_WXYZ_B,:);
w_RMSE = sqrt(mean((w_error).^2,2));

R_state = quat2rotm(x(sn.STATE_Q,:)');
tv_R_state = squeeze(R_state(:,3,:));
tv_spline = [s_x(sn.SPLINE_ACC,:);s_y(sn.SPLINE_ACC,:);s_z(sn.SPLINE_ACC,:)] +  - g_vec;
tv_S_state = tv_spline./vecnorm(tv_spline,2,1);
tv_error = thrustvec_angle_error( tv_S_state, tv_R_state);

if print_latex_results > 0
    ldf = '%0.3f';
    disp('Latex results aligned as [Max Pos. Error, Pos. RMSE, Omega RMSE] as [x,y,z]:')
    disp(['    ', num2str(max_pos_error(1), ldf), ' & ', num2str(max_pos_error(2), ldf), ' & ', num2str(max_pos_error(3), ldf), ' & ' ...
          num2str(pos_RMSE(1), ldf), ' & ', num2str(pos_RMSE(2), ldf), ' & ', num2str(pos_RMSE(3), ldf), ' & ' ...
          num2str(w_RMSE(1), ldf), ' & ', num2str(w_RMSE(2), ldf), ' & ', num2str(w_RMSE(3), ldf), ' \\']);
    disp(' ')
else
    disp('Max Position Errors: ')
    disp(max_pos_error);
    disp('Position RMSE: ')
    disp(pos_RMSE)
    disp('Omega RMSE: ')
    disp(w_RMSE)
end


%% Plotting

if show_plots > 0
    disp('Plotting...')
    
    %%
    f1 = figure('Renderer','opengl');
        clf;

        title(plot_title_3d)
        hold on;
    %     plot3(ref_traj_x, ref_traj_y, ref_traj_z, 'k--')
        plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
        plot3(x(sn.STATE_X,:), x(sn.STATE_Y,:), x(sn.STATE_Z,:), 'b-');
        scatter3(vias_x(sn.SPLINE_POS,:), vias_y(sn.SPLINE_POS,:), vias_z(sn.SPLINE_POS,:), 'ro')
        scatter3(x0(sn.STATE_X), x0(sn.STATE_Y), x0(sn.STATE_Z), 'bx')
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

    f2 = figure('Renderer','opengl');
        clf;

        set(f2,'defaultAxesColorOrder',[0,0,0;0,0,0]);

        subplot(5,1,1)
            title(plot_title_x)

            hold on;
            yyaxis left;
            plot(t,s_x(1,:), 'r-');
            scatter(t_vias, vias_x(sn.SPLINE_POS,:), 'ro')
            plot(t,x(sn.STATE_X,:), 'b-');
            yyaxis right;
            plot(t,ref_traj_x, 'k--')
            hold off;

            grid on;
            yyaxis left;
            ylabel('Position ($m$)');
            maxlim = max([max(abs(ylim)),1.0]);
            ylim([-maxlim maxlim]);
            yyaxis right;
            ylabel('Sine Referece ($m$)');
            ylim([-maxlim maxlim]);
        subplot(5,1,2)
            hold on;
            plot(t,s_x(2,:), 'r-');
            scatter(t_vias, vias_x(sn.SPLINE_VEL,:), 'ro')
            plot(t,x(sn.STATE_VX,:), 'b-');
            hold off;

            grid on;
            ylabel('Velocity ($m/s$)');
            maxlim = max([max(abs(ylim)),0.5]);
            ylim([-maxlim maxlim]);
        subplot(5,1,3)
            hold on;
            yyaxis left;
            plot(t,pitch_c, 'r-');
            plot(t,pitch, 'b-');
    %         plot(t,atan2(s_x(3,:), g), 'g--');
            yyaxis right;
            scatter(t_vias, vias_x(sn.SPLINE_ACC,:), 'ko')
            plot(t,s_x(3,:), 'k--');
            hold off;

            grid on;
            ytickformat('% .2f');
    %         xlabel('Time (s)');
            yyaxis left;
            ylabel('Pitch (rad)');
            maxlim = max([max(abs(ylim)),pi/16]);
            ylim([-maxlim maxlim]);
            yyaxis right;
            ylabel('Acceleration ($m/s^2$)');
            maxlim = max([max(abs(ylim)),1.0]);
            ylim([-maxlim maxlim]);

        subplot(5,1,4)
            hold on;
            yyaxis left;
            plot(t,c(sn.CONTROL_WY_B,:), 'r-');
            plot(t,x(sn.STATE_WY_B,:), 'b-');
            yyaxis right;
            plot(t,s_x(4,:), 'k--');
            hold off;

            grid on;
            ytickformat('% .2f');
    %         xlabel('Time (s)');
            yyaxis left;
            ylabel('$\omega_{y}$ ($rad/s$)');
            maxlim = max([max(abs(ylim)),0.1]);
            ylim([-maxlim maxlim]);
            yyaxis right;
            ylabel('Jerk ($m/s^3$)');
            maxlim = max(abs(ylim));
            ylim([-maxlim maxlim]);

        subplot(5,1,5)
            hold on;
            yyaxis left;
            plot(t,c(sn.CONTROL_DWY_B,:), 'r-');
            yyaxis right;
            plot(t,s_x(5,:), 'k--');
            hold off;

            grid on;
            ytickformat('% .2f');
            xlabel('Time (s)');
            yyaxis left;
            ylabel('$\dot{\omega}_{y}$ ($rad/s^{2}$)');
            maxlim = max([max(abs(ylim)),0.1]);
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
            scatter(t_vias, vias_y(sn.SPLINE_POS,:), 'ro')
            plot(t,x(sn.STATE_Y,:), 'b-');
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
            scatter(t_vias, vias_y(sn.SPLINE_VEL,:), 'ro')
            plot(t,x(sn.STATE_VY,:), 'b-');
            hold off;

            grid on;
            maxlim = max([max(abs(ylim)),0.5]);
            ylim([-maxlim maxlim]);
            ylabel('Velocity ($m/s$)');

        subplot(5,1,3)
            hold on;
            yyaxis left;
            plot(t,roll_c, 'r-');
            plot(t,roll, 'b-');
    %         plot(t,atan2(-s_y(3,:), g), 'g--');
            yyaxis right;
            scatter(t_vias, vias_y(sn.SPLINE_ACC,:), 'ko')
            plot(t,s_y(3,:), 'k--');
            hold off;

            grid on;
            ytickformat('% .2f');
    %         xlabel('Time ($s$)');
            yyaxis left;
            ylabel('Roll ($rad$)');
            maxlim = max([max(abs(ylim)),pi/16]);
            ylim([-maxlim maxlim]);
            yyaxis right;
            ylabel('Acceleration ($m/s^2$)');
            maxlim = max([max(abs(ylim)),1.0]);
            ylim([-maxlim maxlim]);

        subplot(5,1,4)
            hold on;
            yyaxis left;
            plot(t,c(sn.CONTROL_WX_B,:), 'r-');
            plot(t,x(sn.STATE_WX_B,:), 'b-');
            yyaxis right;
            plot(t,s_y(4,:), 'k--');
            hold off;

            grid on;
            ytickformat('% .2f');
    %         xlabel('Time ($s$)');
            yyaxis left;
            ylabel('$\omega_{x}$ ($rad/s$)');
            maxlim = max([max(abs(ylim)),0.1]);
            ylim([-maxlim maxlim]);
            yyaxis right;
            ylabel('Jerk ($m/s^3$)');
            maxlim = max(abs(ylim));
            ylim([-maxlim maxlim]);

        subplot(5,1,5)
            hold on;
            yyaxis left;
            plot(t,c(sn.CONTROL_DWX_B,:), 'r-');
            yyaxis right;
            plot(t,s_y(5,:), 'k--');
            hold off;

            grid on;
            ytickformat('% .2f');
            xlabel('Time ($s$)');
            yyaxis left;
            ylabel('$\dot{\omega}_{x}$ ($rad/s^2$)');
            maxlim = max([max(abs(ylim)),0.1]);
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
            scatter(t_vias, vias_z(sn.SPLINE_POS,:), 'ro')
            plot(t,x(sn.STATE_Z,:), 'b-');
            hold off;

            grid on;
            ylabel('Position ($m$)');
            maxlim = max([max(abs(ylim)),1.5]);
            ylim([-maxlim maxlim]);

        subplot(5,1,2)
            hold on;
            plot(t,s_z(2,:), 'r-');
            scatter(t_vias, vias_z(sn.SPLINE_VEL,:), 'ro')
            plot(t,x(sn.STATE_VZ,:), 'b-');
            hold off;

            grid on;
            ylabel('Velocity ($m/s$)');
            maxlim = max([max(abs(ylim)),0.5]);
            ylim([-maxlim maxlim]);

        subplot(5,1,3)
            hold on;
            plot(t,s_z(3,:), 'r-');
            scatter(t_vias, vias_z(sn.SPLINE_ACC,:), 'ro')
            hold off;

            grid on;
            ytickformat('% .2f');
    %         xlabel('Time (s)');
            ylabel('Acceleration ($m/s^2$)');
            maxlim = max([max(abs(ylim)),1.0]);
            ylim([-maxlim maxlim]);

        subplot(5,1,4)
            title(plot_title_psi)

            hold on;
            plot(t,yaw_c, 'r-');
            scatter(t_vias, wrapToPi(vias_psi(sn.SPLINE_POS,:)), 'ro')
            plot(t,yaw, 'b-');
            hold off;

            grid on;
            ytickformat('% .2f');
    %         xlabel('Time ($s$)');
            ylabel('$\psi$ ($rad$)');
            maxlim = max([max(abs(ylim)),0.1]);
            ylim([-maxlim maxlim]);

        subplot(5,1,5)
            hold on;
            plot(t,c(sn.CONTROL_WZ_B,:), 'r-');
            scatter(t_vias, vias_psi(sn.SPLINE_VEL,:), 'ro')
            plot(t,x(sn.STATE_WZ_B,:), 'b-');
            hold off;

            grid on;
            ytickformat('% .2f');
            xlabel('Time ($s$)');
            ylabel('$\dot{\psi}$ ($rad/s$)');
            maxlim = max([max(abs(ylim)),0.1]);
            ylim([-maxlim maxlim]);

%%
f5 = cell(n,1);
for i = 1:n
    f5{i} = figure('Renderer','opengl');
        clf;

        set(f5{i},'defaultAxesColorOrder',[0,0,0;0,0,0]);
        subplot(3,1,1)
            title([plot_title_r, ' ($\mathbf{r_{', num2str(i), '}}$ Axis)}'])

            hold on;
            plot(t,s_r{i}(sn.SPLINE_POS,:), 'r-');
            scatter(t_vias, vias_r{i}(sn.SPLINE_POS,:), 'ro')
            plot(t,x(sn.STATE_R(1)+(i-1),:), 'b-');
            hold off;

            grid on;
            ylabel('Position ($rad$)');
            maxlim = max(abs(ylim));
            ylim([-maxlim maxlim]);
        subplot(3,1,2)
            hold on;
            plot(t,s_r{i}(sn.SPLINE_VEL,:), 'r-');
            scatter(t_vias, vias_r{i}(sn.SPLINE_VEL,:), 'ro')
            plot(t,x(sn.STATE_RD(1)+(i-1),:), 'b-');
            hold off;

            grid on;
            ylabel('Velocity ($rad/s$)');
            maxlim = max(abs(ylim));
            ylim([-maxlim maxlim]);
        subplot(3,1,3)
            hold on;
            plot(t,s_r{i}(sn.SPLINE_ACC,:), 'r-');
            scatter(t_vias, vias_r{i}(sn.SPLINE_ACC,:), 'ro')
            hold off;

            grid on;
            ylabel('Acceleration ($rad/s/s$)');
            maxlim = max(abs(ylim));
            ylim([-maxlim maxlim]);
end
            
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
    
    f6 = figure('Renderer','opengl');
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
        plot3(ax, x(sn.STATE_X,1:i), x(sn.STATE_Y,1:i), x(sn.STATE_Z,1:i), 'b-');
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
    
    f7 = figure('Renderer','opengl');
    
%     ax1 = subplot(2,1,1);
    ax1 = axes();

        cla;
        plot3(ax1, s_x(1,:), s_y(1,:), s_z(1,:), 'r-');
        hold on;
        plot3(ax1, x(sn.STATE_X,:), x(sn.STATE_Y,:), x(sn.STATE_Z,:), 'b-');

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
        
    f8 = figure('Renderer','opengl');  
%     ax2 = subplot(2,1,2);
    ax2 = axes();

        cla;
        hold on;
        for i = 1:kf_tstep:kf_tf_i   
            xstill = x(:,i);
            xstill(sn.STATE_X) = 0;
            xstill(sn.STATE_Y) = t(i);
            xstill(sn.STATE_Z) = 0;
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

%%

if show_full_animation > 0
    %%
    showmotion( model_d, ...
                t, ...
                fbanim(x([sn.STATE_Q,sn.STATE_XYZ],:), x(sn.STATE_R,:)) );
    
end





