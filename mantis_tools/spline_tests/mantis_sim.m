%% Setup

close all;
clear;
clc;

addpath( genpath('./spatial_v2') );
set(0,'defaultTextInterpreter','latex');

%%
% Print results in latex tablular format if >0 (normally printed otherwise)
print_latex_results = 0;

% Plotting Configuration
do_plotting = 1;
% Enables plots if >0
plotting.show_plots = 0;
% Enables animation if >0
% Also acts as a speed multiplier, 0.5 will be half animation speed
plotting.show_animation = 0.0; %1.0;
plotting.save_animation = 0;
% Enables keyframe view of animation
% Also acts as the keyframe interval, 1.0 will be a keyframe every second
plotting.show_keyframes =0.2; %0.2;
plotting.keyframe_tf = 1.7; % If greater than 0, will stop showing keyframes at kf_tf
% Enable the full 3D animation from the spatial_v2 library if >0
% Camera setup for this animation as well
plotting.show_full_animation = 0;
plotting.camera.body = 0; % Set to 6 to track the robot base
plotting.camera.direction = [-3 -3 2];	% Downwards better viewing angle
plotting.camera.zoom = 0.2;


%% User Variables

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

configs = cell(0,1);
configs{1} = gen_config(0, 1/1000, 1/250, 10, ...
                        'quad_x4', 'serial', 2, ...
                        9, 9, 'fdcc', 'hover', {'steady_90';'steady_0'}, ...
                        'npid_px4', 0, 0.6, deg2rad(30), ...
                        2, 20, 4);

% Bad starting states:
% x0(sn.STATE_Q) = eul2quat([pi/2,0,0])';    % Half yaw error rotation (World)
% x0(sn.STATE_Q) = eul2quat([0,0,deg2rad(179)])';    % (Almost) Full roll error rotation (World)
% x0(sn.STATE_XYZ) = [1;1;0];    % Positional Error (World)
% x0(sn.STATE_R) = zeros(n,1);   % Arm down Joint Error (World)
x0_overrides = cell(0,2);
% x0_overrides(1,:) = {'STATE_Q', eul2quat([pi/2,0,0])'};
x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};


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
    
    disp('----------------');
    disp(' ')
end

results = cell(size(configs));
analyses = cell(size(configs));

for i = 1:size(configs,1)
    % Add the lookup to the workspace for the user
    % This will only be the last one, but at least it allows for some
    % interraction if n is equal for all models
    sn = state_names_lookup(configs{i}.model.n);
    
    %% Run Simulation
    results{i} = mantis_sim_simulate( configs{i}, x0_overrides, plotting.camera );
    
    
    %% Data Analysis
    analyses{i} = mantis_sim_analyse( configs{i}, results{i}, print_latex_results );
    
    
    %% Plotting
    if do_plotting > 0
        mantis_sim_plot(configs{i}, results{i}, analyses{i}, plotting);
        if size(configs,1) > 1
            input('Press enter to continue...');
        end
    end

    
end










