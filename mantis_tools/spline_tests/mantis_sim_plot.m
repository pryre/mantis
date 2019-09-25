function [ fig_handles ] = mantis_sim_plot( config, results, analysis, plotting, config_i)
%MANTIS_SIM_PLOT Plots simulation results and analysis data
%   Detailed explanation goes here

    sn = state_names_lookup(config.model.n);
    fc = 0;
    if ~exist('./figures', 'dir')
        mkdir('./figures');
    end
    
    if config.spline.order == 3
        name_order = 'Cubic';
    elseif config.spline.order == 5
        name_order = 'Quintic';
    elseif config.spline.order == 7
        name_order = 'Septic';
    elseif config.spline.order == 9
        name_order = 'Nonic';
    else
        error(['Unknown plot title for order (', num2str(order)])
    end

    if strcmp(config.spline.tname_base, 'hover')
        name_traj = 'Hover';
    elseif strcmp(config.spline.tname_base, 'yaw_only')
        name_traj = '$\mathbf{\psi}$-Only';
    elseif strcmp(config.spline.tname_base, 'x_only')
        name_traj = 'X-Only';
    elseif strcmp(config.spline.tname_base, 'y_only')
        name_traj = 'Y-Only';
    elseif strcmp(config.spline.tname_base, 'z_only')
        name_traj = 'Z-Only';
    elseif strcmp(config.spline.tname_base, 'circle_flat')
        name_traj = 'XY Circle';
    elseif strcmp(config.spline.tname_base, 'circle_flat_yaw')
        name_traj = 'XY$\mathbf{\psi}$ Circle';
    elseif strcmp(config.spline.tname_base, 'circle_raised')
        name_traj = 'XYZ Corkscrew';
    elseif strcmp(config.spline.tname_base, 'circle_raised_yaw')
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

%     ref_traj_x = cos(linspace(0,2*pi,results.traj.num_traj_points*results.traj.via_steps + 1));
%     ref_traj_y = sin(linspace(0,2*pi,results.traj.num_traj_points*results.traj.via_steps + 1));
%     ref_traj_z = linspace(0,5,results.traj.num_traj_points*results.traj.via_steps + 1);


    if plotting.show_plots > 0
        disp('Plotting...')

        %%
        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');
            clf;

            title(plot_title_3d)
            hold on;
        %     plot3(ref_traj_x, ref_traj_y, ref_traj_z, 'k--')
            plot3(results.traj.s_x(1,:), results.traj.s_y(1,:), results.traj.s_z(1,:), 'r-');
            plot3(results.x(sn.STATE_X,:), results.x(sn.STATE_Y,:), results.x(sn.STATE_Z,:), 'b-');
            scatter3(results.traj.vias_x(sn.SPLINE_POS,:), results.traj.vias_y(sn.SPLINE_POS,:), results.traj.vias_z(sn.SPLINE_POS,:), 'ro')
            scatter3(results.x(sn.STATE_X,1), results.x(sn.STATE_Y,1), results.x(sn.STATE_Z,1), 'bx')
            quiver3(results.traj.vpx, results.traj.vpy, results.traj.vpz, analysis.traj.ref_traj_dir(1,:), analysis.traj.ref_traj_dir(2,:), analysis.traj.ref_traj_dir(3,:), 'r')
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
        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');
            clf;

            set(fig_handles{fc},'defaultAxesColorOrder',[0,0,0;0,0,0]);

            subplot(5,1,1)
                title(plot_title_x)

                hold on;
                yyaxis left;
                plot(results.t,results.traj.s_x(1,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_x(sn.SPLINE_POS,:), 'ro')
                plot(results.t,results.x(sn.STATE_X,:), 'b-');
%                 yyaxis right;
%                 plot(results.t,ref_traj_x, 'k--')
                hold off;

                grid on;
                yyaxis left;
                ylabel('Position ($m$)');
                maxlim = max([max(abs(ylim)),1.0]);
                ylim([-maxlim maxlim]);
%                 yyaxis right;
%                 ylabel('Sine Referece ($m$)');
%                 ylim([-maxlim maxlim]);
            subplot(5,1,2)
                hold on;
                plot(results.t,results.traj.s_x(2,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_x(sn.SPLINE_VEL,:), 'ro')
                plot(results.t,results.x(sn.STATE_VX,:), 'b-');
                hold off;

                grid on;
                ylabel('Velocity ($m/s$)');
                maxlim = max([max(abs(ylim)),0.5]);
                ylim([-maxlim maxlim]);
            subplot(5,1,3)
                hold on;
                yyaxis left;
                plot(results.t,analysis.pitch_c, 'r-');
                plot(results.t,analysis.pitch, 'b-');
        %         plot(t,atan2(s_x(3,:), g), 'g--');
                yyaxis right;
                scatter(results.traj.t_vias, results.traj.vias_x(sn.SPLINE_ACC,:), 'ko')
                plot(results.t,results.traj.s_x(3,:), 'k--');
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
                plot(results.t,results.c(sn.CONTROL_WY_B,:), 'r-');
                plot(results.t,results.x(sn.STATE_WY_B,:), 'b-');
                yyaxis right;
                plot(results.t,results.traj.s_x(4,:), 'k--');
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
                plot(results.t,results.c(sn.CONTROL_DWY_B,:), 'r-');
                yyaxis right;
                plot(results.t,results.traj.s_x(5,:), 'k--');
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
        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');
            clf;

            set(fig_handles{fc},'defaultAxesColorOrder',[0,0,0;0,0,0]);

            subplot(5,1,1)
                title(plot_title_y)

                hold on;
                yyaxis left;
                plot(results.t,results.traj.s_y(1,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_y(sn.SPLINE_POS,:), 'ro')
                plot(results.t,results.x(sn.STATE_Y,:), 'b-');
%                 yyaxis right;
%                 plot(results.t,ref_traj_y, 'k--')
%                 hold off;

                grid on;
                yyaxis left;
                ylabel('Position ($m$)');
                maxlim = max(abs(ylim));
                ylim([-maxlim maxlim]);
%                 yyaxis right;
%                 ylabel('Cosine Referece ($m$)');
%                 ylim([-maxlim maxlim]);
            subplot(5,1,2)
                hold on;
                plot(results.t,results.traj.s_y(2,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_y(sn.SPLINE_VEL,:), 'ro')
                plot(results.t,results.x(sn.STATE_VY,:), 'b-');
                hold off;

                grid on;
                maxlim = max([max(abs(ylim)),0.5]);
                ylim([-maxlim maxlim]);
                ylabel('Velocity ($m/s$)');

            subplot(5,1,3)
                hold on;
                yyaxis left;
                plot(results.t,analysis.roll_c, 'r-');
                plot(results.t,analysis.roll, 'b-');
        %         plot(t,atan2(-s_y(3,:), g), 'g--');
                yyaxis right;
                scatter(results.traj.t_vias, results.traj.vias_y(sn.SPLINE_ACC,:), 'ko')
                plot(results.t,results.traj.s_y(3,:), 'k--');
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
                plot(results.t,results.c(sn.CONTROL_WX_B,:), 'r-');
                plot(results.t,results.x(sn.STATE_WX_B,:), 'b-');
                yyaxis right;
                plot(results.t,results.traj.s_y(4,:), 'k--');
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
                plot(results.t,results.c(sn.CONTROL_DWX_B,:), 'r-');
                yyaxis right;
                plot(results.t,results.traj.s_y(5,:), 'k--');
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
        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');
            clf;

            set(fig_handles{fc},'defaultAxesColorOrder',[0,0,0;0,0,0]);

            subplot(5,1,1)
                title(plot_title_z)

                hold on;
                plot(results.t,results.traj.s_z(1,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_z(sn.SPLINE_POS,:), 'ro')
                plot(results.t,results.x(sn.STATE_Z,:), 'b-');
                hold off;

                grid on;
                ylabel('Position ($m$)');
                maxlim = max([max(abs(ylim)),1.5]);
                ylim([-maxlim maxlim]);

            subplot(5,1,2)
                hold on;
                plot(results.t,results.traj.s_z(2,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_z(sn.SPLINE_VEL,:), 'ro')
                plot(results.t,results.x(sn.STATE_VZ,:), 'b-');
                hold off;

                grid on;
                ylabel('Velocity ($m/s$)');
                maxlim = max([max(abs(ylim)),0.5]);
                ylim([-maxlim maxlim]);

            subplot(5,1,3)
                hold on;
                plot(results.t,results.traj.s_z(3,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_z(sn.SPLINE_ACC,:), 'ro')
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
                plot(results.t,analysis.yaw_c, 'r-');
                scatter(results.traj.t_vias, wrapToPi(results.traj.vias_psi(sn.SPLINE_POS,:)), 'ro')
                plot(results.t,analysis.yaw, 'b-');
                hold off;

                grid on;
                ytickformat('% .2f');
        %         xlabel('Time ($s$)');
                ylabel('$\psi$ ($rad$)');
                maxlim = max([max(abs(ylim)),0.1]);
                ylim([-maxlim maxlim]);

            subplot(5,1,5)
                hold on;
                plot(results.t,results.c(sn.CONTROL_WZ_B,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_psi(sn.SPLINE_VEL,:), 'ro')
                plot(results.t,results.x(sn.STATE_WZ_B,:), 'b-');
                hold off;

                grid on;
                ytickformat('% .2f');
                xlabel('Time ($s$)');
                ylabel('$\dot{\psi}$ ($rad/s$)');
                maxlim = max([max(abs(ylim)),0.1]);
                ylim([-maxlim maxlim]);

    %%
    fc = fc + 1;
    fig_handles{fc} = cell(config.model.n,1);
    for i = 1:config.model.n
        fig_handles{fc}{i} = figure('Renderer','opengl');
            clf;

            set(fig_handles{fc}{i},'defaultAxesColorOrder',[0,0,0;0,0,0]);
            subplot(3,1,1)
                title([plot_title_r, ' ($\mathbf{r_{', num2str(i), '}}$ Axis)}'])

                hold on;
                plot(results.t,results.traj.s_r{i}(sn.SPLINE_POS,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_r{i}(sn.SPLINE_POS,:), 'ro')
                plot(results.t,results.x(sn.STATE_R(1)+(i-1),:), 'b-');
                hold off;

                grid on;
                ylabel('Position ($rad$)');
                maxlim = max(abs(ylim));
                ylim([-maxlim maxlim]);
            subplot(3,1,2)
                hold on;
                plot(results.t,results.traj.s_r{i}(sn.SPLINE_VEL,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_r{i}(sn.SPLINE_VEL,:), 'ro')
                plot(results.t,results.x(sn.STATE_RD(1)+(i-1),:), 'b-');
                hold off;

                grid on;
                ylabel('Velocity ($rad/s$)');
                maxlim = max(abs(ylim));
                ylim([-maxlim maxlim]);
            subplot(3,1,3)
                hold on;
                plot(results.t,results.traj.s_r{i}(sn.SPLINE_ACC,:), 'r-');
                scatter(results.traj.t_vias, results.traj.vias_r{i}(sn.SPLINE_ACC,:), 'ro')
                hold off;

                grid on;
                ylabel('Acceleration ($rad/s/s$)');
                maxlim = max(abs(ylim));
                ylim([-maxlim maxlim]);
    end

    % print(f1, ['./figures/',plot_titles{2}], '-depsc')
    end

    %%
    if plotting.show_animation > 0
        flight_space = 3.5; %3.5; 6.0; 9.0;
        frame_size = 0.45;
        prop_rad = 0.075;

        framerate = 30; %Hz
        sim_rate = framerate/plotting.show_animation; %Hz
        % sim_tstep needs to be a positive integer, might not be if dt is not a
        % multiple of sim_rate
        sim_tstep = round(1/(config.time.dt*sim_rate));
        
        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');
        ax = axes();

        axis([-flight_space,flight_space,-flight_space,flight_space,-flight_space,flight_space]);
        axis square;
        grid on;
    %     ax.Units = 'pixels';
    %     axpos = ax.Position;
    %     axti = ax.TightInset;
    %     axrect = [-axti(1), -axti(2), axpos(3)+axti(1)+axti(3), axpos(4)+axti(2)+axti(4)];
    
        vid_write = VideoWriter(['./figures/animation_', num2str(config_i)], 'Archival');
    %     vid_write.Quality = 95;
        vid_write.FrameRate = framerate;
        if plotting.save_animation > 0
            open(vid_write);
        end

        for i = 1:sim_tstep:length(results.t)
            tic;

%             cla;
            plot3(ax, results.traj.s_x(1,:), results.traj.s_y(1,:), results.traj.s_z(1,:), 'r-');

            hold on;
            plot3(ax, results.x(sn.STATE_X,1:i), results.x(sn.STATE_Y,1:i), results.x(sn.STATE_Z,1:i), 'b-');
            draw_quad( ax, results.x(:,i), frame_size, prop_rad ); % Draw last so it's on top
            hold off;

            view(ax,55,35);
            axis(ax,[-flight_space,flight_space,-flight_space,flight_space,-flight_space,flight_space]);
            axis square;
            grid on;

            drawnow;

            if plotting.save_animation > 0
    %             writeVideo(vid_write,getframe(ax,axrect))
                writeVideo(vid_write,getframe(ax));
            else
                t_now = toc;
                pause((1/20) - t_now);
            end

        end

        if plotting.save_animation > 0
            close(vid_write);
            disp(['Animation writen to: ', vid_write.Path])
            disp('Convert animation to MP4 with: ffmpeg -i animation.mj2 -q:v 0 animation.mp4')
        end

        disp('Finished animation!')
    end

    %%
    if plotting.show_keyframes > 0
        flight_space = 2.2; %3.5; 6.0; 9.0;
        frame_size = 0.45;
        prop_rad = 0.075;

        still_scaler = 0.2;

        still_frame_size = still_scaler*frame_size;
        still_prop_rad = still_scaler*prop_rad;

        kf_tf = tf;
        if plotting.keyframe_tf > 0
            kf_tf = plotting.keyframe_tf;
        end
        kf_tf_i = find(results.t <= kf_tf,1,'last');

        kf_tstep = round((1/config.time.dt)*plotting.show_keyframes);

        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');

    %     ax1 = subplot(2,1,1);
        ax1 = axes();

            cla;
            plot3(ax1, results.traj.s_x(1,:), results.traj.s_y(1,:), results.traj.s_z(1,:), 'r-');
            hold on;
            plot3(ax1, results.x(sn.STATE_X,:), results.x(sn.STATE_Y,:), results.x(sn.STATE_Z,:), 'b-');

            for i = 1:kf_tstep:kf_tf_i
                draw_quad( ax1, results.x(:,i), frame_size, prop_rad );

            end
            hold off;

            axis([-flight_space,flight_space,-flight_space,flight_space,-flight_space,flight_space]);
            axis square;
            grid on;
            xlabel('X Position ($m$)');
            ylabel('Y Position ($m$)');
            zlabel('Z Position ($m$)');
            view(55,35);

        fc = fc + 1;
        fig_handles{fc} = figure('Renderer','opengl');  
    %     ax2 = subplot(2,1,2);
        ax2 = axes();

            cla;
            hold on;
            for i = 1:kf_tstep:kf_tf_i
                xstill = results.x(:,i);
                xstill(sn.STATE_X) = 0;
                xstill(sn.STATE_Y) = results.t(i);
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

    if plotting.show_full_animation > 0
        showmotion( results.model_d, ...
                    results.t, ...
                    fbanim(results.x([sn.STATE_Q,sn.STATE_XYZ],:), results.x(sn.STATE_R,:)) );
    end

end

