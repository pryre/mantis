%% Setup

close all;
clear;
clc;

set(0,'defaulttextInterpreter','latex');


%% User Variables
% Spline Order:
%   cubic:  3
%   quntic: 5
%   septic: 7
%   nonic:  9
order = 9;

t0 = 0;
dt = 1/2000;
tf = 10;

num_via_points = 9;

% 'psi_only'
% 'x_only'
% 'y_only'
% 'z_only'
% 'circle_flat'
% 'circle_flat_yaw'
% 'circle_raised'
% 'circle_raised_yaw'
tname = 'circle_raised_yaw';

% via_est_method = 'linear';
via_est_method = 'fdcc';


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

SPLINE_POS = 1;
SPLINE_VEL = 2;
SPLINE_ACC = 3;
SPLINE_JERK = 4;
SPLINE_SNAP = 5;

g = 9.80665;
g_vec = [0;0;-g];

t = t0:dt:tf;
t_vias = linspace(t0,tf,num_via_points);
sd = t_vias(2) - t_vias(1);
vias_x = gen_vias(via_est_method, vpx, sd);
vias_y = gen_vias(via_est_method, vpy, sd);
vias_z = gen_vias(via_est_method, vpz, sd);
vias_psi = gen_vias(via_est_method, vppsi, sd);
[st, s_x] = gen_spline(t0, tf, vias_x, order, via_steps);
[~, s_y] = gen_spline(t0, tf, vias_y, order, via_steps);
[~, s_z] = gen_spline(t0, tf, vias_z, order, via_steps);
[~, s_psi] = gen_spline(t0, tf, vias_psi, order, via_steps);

if st ~= t %#ok<BDSCI>
    error('Error constructing spline times')
end

%% Plotting
disp('Plotting...')

ref_traj_dir = zeros(3,length(vias_psi(SPLINE_POS,:)));
yawR = eul2rotm([vias_psi(SPLINE_POS,:);
                 zeros(size(vias_psi(SPLINE_POS,:)));
                 zeros(size(vias_psi(SPLINE_POS,:)))]');
for ir = 1:length(vias_psi(SPLINE_POS,:))
    ref_traj_dir(:,ir) = yawR(:,:,ir)*[1;0;0];
end 

as = 1.5;

f1 = figure();
    clf;
% 
%     set(f1,'defaultAxesColorOrder',[0,0,0;0,0,0]);
    subplot(2,1,1)
        title('\textbf{3D Position and Heading Trajectory}')
        hold on;
        plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'b-');
        scatter3(vias_x(SPLINE_POS,:), vias_y(SPLINE_POS,:), vias_z(SPLINE_POS,:), 'ro')
        quiver3(vpx, vpy, vpz, ref_traj_dir(1,:), ref_traj_dir(2,:), ref_traj_dir(3,:), 0.2, 'k')
        hold off;

        grid on;
        xlabel('X Position (\(m\))');
        ylabel('Y Position (\(m\))');
        zlabel('Z Position (\(m\))');
        axis([-as, as, -as, as, 0, 2*as])
        axis('equal')
        view(-45,40)
   
    subplot(4,2,5)

        hold on;
        plot(t,s_x(1,:), 'b-');
        scatter(t_vias, vias_x(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
        xlabel('Time (\(s\))');
        ylabel('X Position (\(m\))');
        ylim([-as, as]);

    subplot(4,2,6)
        hold on;
        plot(t,s_y(1,:), 'b-');
        scatter(t_vias, vias_y(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
        xlabel('Time (\(s\))');
        ylabel('Y Position (\(m\))');
        ylim([-as, as]);

    subplot(4,2,7)
        hold on;
        plot(t,s_z(1,:), 'b-');
        scatter(t_vias, vias_z(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
        xlabel('Time (\(s\))');
        ylabel('Z Position (\(m\))');
        maxlim = max(abs(ylim));
%         ylim([0 maxlim+1]);
        ylim([1/3 14/3]);
        
    subplot(4,2,8)
        hold on;
        plot(t,s_psi(1,:), 'k-');
        scatter(t_vias, vias_psi(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
        xlabel('Time (\(s\))');
        ylabel('Heading (\(rad\))');
%         maxlim = max(abs(ylim));
%         ylim([-maxlim maxlim]);
%         ylim([-pi/4, 9*pi/4]);
        ylim([-pi/2, 5*pi/2]);
        yticks([0, pi, 2*pi])
        yticklabels({'0', '\pi','2\pi'})
%         yticks(-pi/2:pi/2:2*pi)
%         yticklabels({'-\pi/2','0','\pi/2', '\pi','3\pi/2','2\pi'})
        %%
print(f1, './traj_ref_simple_vert.eps', '-depsc')
print(f1, './traj_ref_simple_vert.png', '-dpng', '-r720')