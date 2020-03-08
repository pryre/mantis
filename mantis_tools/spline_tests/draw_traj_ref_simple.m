% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

%% Setup

close all;
clear;
clc;

set(0,'defaulttextInterpreter','latex');
show_animation = 1;
save_animation = 1;


%% User Variables
% Spline Order:
%   cubic:  3
%   quntic: 5
%   septic: 7
%   nonic:  9
order = 9;

t0 = 0;
dt = 1/200;
tf = 5;

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
[vpx, vpy, vpz, vppsi] = gen_trajectory_vias(tname, num_via_points);

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
% sd = t_vias(2) - t_vias(1);
vias_x = gen_vias(via_est_method, vpx, t_vias);
vias_y = gen_vias(via_est_method, vpy, t_vias);
vias_z = gen_vias(via_est_method, vpz, t_vias);
vias_psi = gen_vias(via_est_method, vppsi, t_vias);
[st, s_x] = gen_spline(t_vias, vias_x, order, dt);
[~, s_y] = gen_spline(t_vias, vias_y, order, dt);
[~, s_z] = gen_spline(t_vias, vias_z, order, dt);
[~, s_psi] = gen_spline(t_vias, vias_psi, order, dt);

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

f1 = figure('Position', [10 10 500 707]);
    clf;
% 
%     set(f1,'defaultAxesColorOrder',[0,0,0;0,0,0]);
    subplot(2,1,1)
        title('\textbf{3D Position and Heading Trajectory}')
        hold on;
        plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'b-');
        scatter3(vias_x(SPLINE_POS,:), vias_y(SPLINE_POS,:), vias_z(SPLINE_POS,:), 'ro')
        quiver3(vpx, vpy, vpz, ref_traj_dir(1,:), ref_traj_dir(2,:), ref_traj_dir(3,:), 0.2, 'r')
        hold off;

        grid on;
        xlabel('X (\(m\))');
        ylabel('Y (\(m\))');
        zlabel('Z (\(m\))');
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
%         xlabel('Time (\(s\))');
        ylabel('X Position (\(m\))');
        ylim([-as, as]);

    subplot(4,2,6)
        hold on;
        plot(t,s_y(1,:), 'b-');
        scatter(t_vias, vias_y(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
%         xlabel('Time (\(s\))');
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
        plot(t,s_psi(1,:), 'b-');
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
% print(f1, './traj_ref_simple_vert.eps', '-depsc')
% print(f1, './traj_ref_simple_vert.png', '-dpng', '-r720')


%%
f2 = figure('Position', [10 10 1200 600]);
    clf;
% 
%     set(f1,'defaultAxesColorOrder',[0,0,0;0,0,0]);
    subplot(1,2,2)
        title('\textbf{3D Position and Heading Trajectory}')
        hold on;
        plot3(s_x(1,:), s_y(1,:), s_z(1,:), 'b-');
        scatter3(vias_x(SPLINE_POS,:), vias_y(SPLINE_POS,:), vias_z(SPLINE_POS,:), 'ro')
        quiver3(vpx, vpy, vpz, ref_traj_dir(1,:), ref_traj_dir(2,:), ref_traj_dir(3,:), 0.2, 'r')
        hold off;

        grid on;
        xlabel('X (\(m\))');
        ylabel('Y (\(m\))');
        zlabel('Z (\(m\))');
        axis([-as, as, -as, as, 0, 2*as])
        axis('equal')
        view(-45,40)
   
    subplot(2,4,1)

        hold on;
        plot(t,s_x(1,:), 'b-');
        scatter(t_vias, vias_x(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
%         xlabel('Time (\(s\))');
        ylabel('X Position (\(m\))');
        ylim([-as, as]);

    subplot(2,4,2)
        hold on;
        plot(t,s_y(1,:), 'b-');
        scatter(t_vias, vias_y(SPLINE_POS,:), 'ro')
        hold off;

        grid on;
        ytickformat('% .1f');
%         xlabel('Time (\(s\))');
        ylabel('Y Position (\(m\))');
        ylim([-as, as]);

    subplot(2,4,5)
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
        
    subplot(2,4,6)
        hold on;
        plot(t,s_psi(1,:), 'b-');
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
print(f2, './figures/traj_ref_simple_horz.eps', '-depsc')
print(f2, './figures/traj_ref_simple_horz.png', '-dpng', '-r720')

%%

if logical(show_animation)
    fc = figure('Position', [10 10 1920 1080]);

    framerate = 30; %Hz
    sim_rate = framerate/show_animation; %Hz
    % sim_tstep needs to be a positive integer, might not be if dt is not a
    % multiple of sim_rate
    sim_tstep = round(1/(dt*sim_rate));

    vid_write = VideoWriter('./figures/animation_traj_ref_simple', 'Uncompressed AVI');
%     vid_write.Quality = 95;
    vid_write.FrameRate = framerate;
    if logical(save_animation) > 0
        open(vid_write);
    end
    
    for i = 1:sim_tstep:length(t)
        tic;

        subplot(1,2,2)
            plot3(s_x(SPLINE_POS,:), s_y(SPLINE_POS,:), s_z(SPLINE_POS,:), 'b-');
            hold on;
            scatter3(vias_x(SPLINE_POS,:), vias_y(SPLINE_POS,:), vias_z(SPLINE_POS,:), 'ro')
            quiver3(vpx, vpy, vpz, ref_traj_dir(1,:), ref_traj_dir(2,:), ref_traj_dir(3,:), 0.2, 'r')
            
            plot3(s_x(1,i), s_y(1,i), s_z(1,i), 'ko', 'MarkerFaceColor', 'k');
            
            yawR_track = eul2rotm([s_psi(SPLINE_POS,i);0;0]');
            ref_track_dir = yawR_track*[1;0;0];
            quiver3(s_x(SPLINE_POS,i), s_y(SPLINE_POS,i), s_z(SPLINE_POS,i), ref_track_dir(1), ref_track_dir(2), ref_track_dir(3), 0.2, 'k')

            hold off;

            grid on;
            xlabel('X (\(m\))');
            ylabel('Y (\(m\))');
            zlabel('Z (\(m\))');
            axis([-as, as, -as, as, 0, 2*as])
            axis('equal')
            view(-45,40)

        subplot(2,4,1)

            plot(t,s_x(SPLINE_POS,:), 'b-');
            hold on;
            scatter(t_vias, vias_x(SPLINE_POS,:), 'ro')
            plot(t(i),s_x(SPLINE_POS,i), 'ko', 'MarkerFaceColor', 'k');
            hold off;

            grid on;
            ytickformat('% .1f');
            ylabel('X Position (\(m\))');
            ylim([-as, as]);

        subplot(2,4,2)
            plot(t,s_y(SPLINE_POS,:), 'b-');
            hold on;
            scatter(t_vias, vias_y(SPLINE_POS,:), 'ro')
            plot(t(i),s_y(SPLINE_POS,i), 'ko', 'MarkerFaceColor', 'k');
            hold off;

            grid on;
            ytickformat('% .1f');
            ylabel('Y Position (\(m\))');
            ylim([-as, as]);

        subplot(2,4,5)
            plot(t,s_z(SPLINE_POS,:), 'b-');
            hold on;
            scatter(t_vias, vias_z(SPLINE_POS,:), 'ro')
            plot(t(i),s_z(SPLINE_POS,i), 'ko', 'MarkerFaceColor', 'k');
            hold off;

            grid on;
            ytickformat('% .1f');
            xlabel('Time (\(s\))');
            ylabel('Z Position (\(m\))');
            maxlim = max(abs(ylim));
            ylim([1/3 14/3]);

        subplot(2,4,6)
            plot(t,s_psi(SPLINE_POS,:), 'b-');
            hold on;
            scatter(t_vias, vias_psi(SPLINE_POS,:), 'ro')
            plot(t(i),s_psi(SPLINE_POS,i), 'ko', 'MarkerFaceColor', 'k');
            hold off;

            grid on;
            ytickformat('% .1f');
            xlabel('Time (\(s\))');
            ylabel('Heading (\(rad\))');
            ylim([-pi/2, 5*pi/2]);
            yticks([0, pi, 2*pi])
            yticklabels({'0', '\pi','2\pi'})

        drawnow;

        if save_animation > 0
            writeVideo(vid_write,getframe(fc));
        else
            t_now = toc;
            pause((1/20) - t_now);
        end

    end

    if logical(save_animation)
        close(vid_write);
        disp(['Animation writen to: ', vid_write.Path])
        disp('Convert animation to MP4 with: ffmpeg -i animation.mj2 -q:v 0 animation.mp4')
        disp('Fix aspect ratio: ffmpeg -i animation.mp4 -aspect 1920:1025 -c copy animation.mp4')
    end

    disp('Finished animation!')
end 