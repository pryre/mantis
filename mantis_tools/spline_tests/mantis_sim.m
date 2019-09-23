%% Setup

close all;
clear;
clc;

addpath( genpath('./spatial_v2') );
set(0,'defaultTextInterpreter','latex');

%%
% Print results in latex tablular format if >0 (normally printed otherwise)
print_latex_results = 1;

% Plotting Configuration
do_plotting = 1;
% Enables plots if >0
plotting.show_plots = 1;
% Enables animation if >0
% Also acts as a speed multiplier, 0.5 will be half animation speed
plotting.show_animation = 0.0; %1.0;
plotting.save_animation = 0;
% Enables keyframe view of animation
% Also acts as the keyframe interval, 1.0 will be a keyframe every second
plotting.show_keyframes =0; %0.2;
plotting.keyframe_tf = 1.7; % If greater than 0, will stop showing keyframes at kf_tf
% Enable the full 3D animation from the spatial_v2 library if >0
% Camera setup for this animation as well
plotting.show_full_animation = 0;
plotting.camera.body = 0; % Set to 6 to track the robot base
plotting.camera.direction = [-3 -3 2];	% Downwards better viewing angle
plotting.camera.zoom = 0.2;


%% User Variables

% Define some preset configurations to keep this script clutter-free
[ configs, x0_overrides ] = gen_test_cases( 'tuning' );


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
    
end

results = cell(size(configs));
analyses = cell(size(configs));

%% Run Simulation
disp('--== Simulation ==--')
for i = 1:size(configs,1)
    % Add the lookup to the workspace for the user
    % This will only be the last one, but at least it allows for some
    % interraction if n is equal for all models
    sn = state_names_lookup(configs{i}.model.n);
    
    disp(['Configuration ', num2str(i)])
    results{i} = mantis_sim_simulate( configs{i}, x0_overrides, plotting.camera );
end

%% Data Analysis
disp('--== Analysis ==--')
for i = 1:size(configs,1)
    disp(['Configuration ', num2str(i)])
    analyses{i} = mantis_sim_analyse( configs{i}, results{i}, print_latex_results );
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










