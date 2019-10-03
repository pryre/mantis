%% Setup

close all;
clear;
clc;

addpath( genpath('./spatial_v2') );
set(0,'defaultTextInterpreter','latex');
% set(0, 'DefaultFigureRenderer', 'painter');

%%

% Print results in latex tablular format if >0
export_latex_analysis = 1;
show_settling_time = 1;

% Plotting Configuration
do_plotting = 0;
plotting.flight_space = [-1,1,-1,1,0,2];
% Overly plot of multiple configs
plotting.show_multiconfig_plot = 1;
plotting.show_multiconfig_list_reference = 0;
plotting.show_multiconfig_end_effector = 0;
plotting.show_multiconfig_quiv_scale = 0.2;
plotting.show_multiconfig_step = 100;
plotting.multiconfig_legends = {'NPID-U', 'CTC-F', 'CTC-U', 'Feed-F', 'Feed-U'};
% Enables plots if >0
plotting.show_plots = 0;
% Enables animation if >0
% Also acts as a speed multiplier, 0.5 will be half animation speed
plotting.show_animation = 0.0; %1.0;
plotting.save_animation = 0;
% Enables keyframe view of animation
% Also acts as the keyframe interval, 1.0 will be a keyframe every second
plotting.show_keyframes =0; %0.2;
plotting.keyframe_tf = 1.7; % If greater than 0, will stop showing keyframes at kf_tf
% Enable the full 3D animation from the spatial_v2 library if >0
% Camera setup for this anim3ation as well
plotting.show_full_animation = 0;
camera.body = 0; % Set to 6 to track the robot base
camera.direction = [-3 -3 2];	% Downwards better viewing angle
camera.zoom = 0.2;


%% User Variables

% % swaymsg 'for_window [title="HG_Peer_OffScreenWindow"] floating enable'
% % swaymsg 'for_window [title="Figure 1"] floating enable'
% % cp ./figures/* ~/Cloudstor/PhD/Thesis/graphics/results/simulated
% copyfile ./figures/* ~/Cloudstor/PhD/Thesis/graphics/results/simulated


% See "help gen_test_cases" for test_name options
% test_name = 'hover_0_0';
% plotting.multiconfig_plot_title = 'Steady Hover (\(r_{1}=r_{2}=0^{\circ}\))';

% test_name = 'hover_90_0';
% plotting.flight_space = [-0.2,0.2,-0.2,0.2,0.8,1.2];
% plotting.show_multiconfig_quiv_scale = 0.1;
% plotting.multiconfig_plot_title = 'Steady Hover (\(r_{1}=90^{\circ},\,r_{2}=0^{\circ}\))';

% test_name = 'full_state_error';
% plotting.multiconfig_plot_title = 'Full State Errors (\(e_{p_{b}}=1,\,e_{\psi_{b}}=e_{r_{1}}=e_{r_{2}}=90^{\circ}\))';

% test_name = 'inversion_0_0';
% plotting.multiconfig_plot_title = 'Inversion Recovery (\(e_{\phi_{b}}=1,\,r_{1}=r_{2}=0^{\circ}\))';

% test_name = 'inversion_90_0';
% plotting.multiconfig_plot_title = 'Inversion Recovery (\(e_{\phi_{b}}=179^{\circ},\,r_{1}=90^{\circ},\,r_{2}=0^{\circ}\))';

% test_name = 'tuning_converging';
% plotting.show_multiconfig_step = 10;
% plotting.flight_space = [-0.6,0.6,-0.6,0.6,0,1.2];
% plotting.multiconfig_plot_title = 'CTC-F High-Level Tracking (Converging)';
% plotting.multiconfig_legends = {'\(\omega_{0p}=1\)', ...
%                                 '\(\omega_{0p}=2\)', ...
%                                 '\(\omega_{0p}=5\)'};

% test_name = 'tuning_diverging';
% plotting.show_multiconfig_step = 10;
% plotting.flight_space = [-0.6,0.6,-0.6,0.6,0,1.2];
% plotting.multiconfig_plot_title = 'CTC-F High-Level Tracking (Diverging)';
% plotting.multiconfig_legends = {'\(\omega_{0p}=8.250\)', ...
%                                 '\(\omega_{0p}=8.625\)'};

% test_name = 'spiral_base_slow';
% show_settling_time = 0;
% plotting.show_multiconfig_list_reference = 1;
% plotting.flight_space = [-2,2,-2,2,0,4];
% plotting.multiconfig_plot_title = 'Vectoring Base Tracking (\(\Delta t = 40s\))';

% test_name = 'spiral_base_quick';
% show_settling_time = 0;
% plotting.show_multiconfig_list_reference = 1;
% plotting.flight_space = [-2,2,-2,2,0,4];
% plotting.multiconfig_plot_title = 'Vectoring Base Tracking (\(\Delta t = 10s\))';

% test_name = 'spiral_end_slow';
% show_settling_time = 0;
% plotting.show_multiconfig_end_effector = 1;
% plotting.show_multiconfig_list_reference = 1;
% plotting.flight_space = [-2,2,-2,2,0,4];
% plotting.multiconfig_plot_title = 'Vectoring End Tracking (\(\Delta t = 40s\))';

test_name = 'spiral_end_quick';
show_settling_time = 0;
plotting.show_multiconfig_end_effector = 1;
plotting.show_multiconfig_list_reference = 1;
plotting.flight_space = [-2,2,-2,2,0,4];
plotting.multiconfig_plot_title = 'Vectoring End Tracking (\(\Delta t = 10s\))';

% Define some preset configurations to keep this script clutter-free
[ configs, x0_overrides ] = gen_test_cases( test_name );


%% Simulation

% Display starting state modifiers
if size(x0_overrides,1) > 0
    disp('--== Start State ==--');
    % Rest of simulator setup
    for i = 1:size(x0_overrides,1)
        n = x0_overrides{i,1};
        v = x0_overrides{i,2};
        if size(v,1) > 1
            v = v';
        end
        disp([ '    -', n, ': ', sprintf('%0.5f ', v);])
    end
    
    disp(' ')
end

results = cell(size(configs));
analyses = cell(size(configs));

%% Run Simulation
disp('--== Simulation ==--')
sn = state_names_lookup(configs{1}.model.n);

% for i = 1:size(configs,1)
parfor i = 1:size(configs,1)
    % Add the lookup to the workspace for the user
    % This will only be the last one, but at least it allows for some
    % interraction if n is equal for all models
    
    disp(['Configuration ', num2str(i)])
    results{i} = mantis_sim_simulate( configs{i}, x0_overrides, camera );
    disp(' ')
end

%% Data Analysis
disp('--== Analysis ==--')

% for j = 1:size(configs,1)
parfor i = 1:size(configs,1)
    disp(['Configuration ', num2str(i)])
    analyses{i} = mantis_sim_analyse( configs{i}, results{i} );
end

if export_latex_analysis > 0
    save_analysis_latex( test_name, configs, analyses, show_settling_time )
end

%% Plotting

if do_plotting > 0
    disp('--== Plotting ==--')
    for i = 1:size(configs,1)
        disp(['Configuration ', num2str(i)])
        h = mantis_sim_plot(configs{i}, results{i}, analyses{i}, plotting, i);
        if size(configs,1) > 1
            input('Press enter to continue...');
%             for j = 1:length(h)
%                 set(h{j},'Resizable', 'off')
%             end
            close all
        end
    end
end
    %%
if plotting.show_multiconfig_plot > 0
    fm = figure();
        clf;

        title(plotting.multiconfig_plot_title)
        hold on;
        % Assume that we have the same reference and x0 for all configs
        plot3(results{1}.c(sn.CONTROL_PX_B,1:plotting.show_multiconfig_step:end), results{1}.c(sn.CONTROL_PY_B,1:plotting.show_multiconfig_step:end), results{1}.c(sn.CONTROL_PZ_B,1:plotting.show_multiconfig_step:end), 'r-');
        if plotting.show_multiconfig_end_effector > 0
            plot3(analyses{i}.end_effector_pos.ref(1,1:plotting.show_multiconfig_step:end), analyses{i}.end_effector_pos.ref(2,1:plotting.show_multiconfig_step:end), analyses{i}.end_effector_pos.ref(3,1:plotting.show_multiconfig_step:end), 'r--');
        end
        scatter3(results{1}.traj.vias_x(sn.SPLINE_POS,:), results{1}.traj.vias_y(sn.SPLINE_POS,:), results{1}.traj.vias_z(sn.SPLINE_POS,:), 'ro')
        quiver3(results{1}.traj.vpx, results{1}.traj.vpy, results{1}.traj.vpz, analyses{1}.traj.ref_traj_dir(1,:), analyses{1}.traj.ref_traj_dir(2,:), analyses{1}.traj.ref_traj_dir(3,:), 'r', 'AutoScaleFactor', plotting.show_multiconfig_quiv_scale)
%         scatter3(results{1}.x(sn.STATE_X,1), results{1}.x(sn.STATE_Y,1), results{1}.x(sn.STATE_Z,1), 'bx')
        
        cmap = lines(size(configs,1));
        
        for i = 1:size(configs,1)
            plot3(results{i}.x(sn.STATE_X,1:plotting.show_multiconfig_step:end), results{i}.x(sn.STATE_Y,1:plotting.show_multiconfig_step:end), results{i}.x(sn.STATE_Z,1:plotting.show_multiconfig_step:end), '-', 'Color', cmap(i,:));
        
            if plotting.show_multiconfig_end_effector
                plot3(analyses{i}.end_effector_pos.state(1,1:plotting.show_multiconfig_step:end), analyses{i}.end_effector_pos.state(2,1:plotting.show_multiconfig_step:end), analyses{i}.end_effector_pos.state(3,1:plotting.show_multiconfig_step:end), '--', 'Color', cmap(i,:));
            end
        end
        
        lp = [];
        leg_names = plotting.multiconfig_legends;
        if plotting.show_multiconfig_list_reference > 0
            lp(end+1) = plot3(nan, nan, nan, 'r-');
            leg_names = [{'Reference'}, leg_names];
        end
        
        % Fake data inputs for legend
        for i = 1:size(configs,1)
            lp(end+1) = plot3(nan, nan, nan, '-', 'Color', cmap(i,:));
        end
        
        if plotting.show_multiconfig_end_effector > 0
            lp(end+1) = plot3(nan, nan, nan, 'k--');
            leg_names(end+1) = {'End Eff.'};
        end
        hold off;
        
        axis('equal')
        axis(plotting.flight_space)
%         xlim([results{1}.x(sn.STATE_X,1)-fa, results{1}.x(sn.STATE_X,1)+fa]);
%         ylim([results{1}.x(sn.STATE_Y,1)-fa, results{1}.x(sn.STATE_Y,1)+fa]);
%         zlim([results{1}.x(sn.STATE_Z,1)-fa, results{1}.x(sn.STATE_Z,1)+fa]);
%         xlim([-plotting.flight_space, plotting.flight_space]);
%         ylim([-plotting.flight_space, plotting.flight_space]);
%         zlim([0, 2*plotting.flight_space]);
        view(55,35);

        grid on;
        xlabel('X Position ($m$)');
        ylabel('Y Position ($m$)');
        zlabel('Z Position ($m$)');
        
        l = legend(lp, leg_names,'Interpreter','latex');
        set(l,'Position',[0.80, ...
                          0.50, ...
                          0.17, ...
                          0.20]);
                      
    set(gca, 'SortMethod', 'ChildOrder');
    set(fm, 'Color', 'w');
                      
%     print(fm, ['./figures/',test_name], '-depsc')
    export_fig(['./figures/',test_name], '-eps', fm)


end









