%% Setup

close all;
clear;
clc;
%%
addpath( genpath('./spatial_v2') );

set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
% set(0,'DefaultColorbarInterpreter','latex');
set(0,'DefaultAxesXGrid','on');
set(0,'DefaultAxesYGrid','on');
set(0,'DefaultAxesZGrid','on');
% set(0,'DefaultAxesSortMethod','ChildOrder');
set(0,'DefaultFigureColor','w');
% set(0, 'DefaultFigureRenderer', 'painter');

%%

camera.body = 0; % Set to 6 to track the robot base
camera.direction = [-3 -3 2];	% Downwards better viewing angle
camera.zoom = 0.2;


%% User Variables

% % swaymsg 'for_window [title="HG_Peer_OffScreenWindow"] floating enable'
% % swaymsg 'for_window [title="Figure 1"] floating enable'
% % cp ./figures/* ~/Cloudstor/PhD/Thesis/graphics/results/simulated
% copyfile ./figures/* ~/Cloudstor/PhD/Thesis/graphics/results/simulated

dt = 1/1000;

default_config = gen_config(0, dt, dt, 1, ...
                            'quad_x4', 'serial', 2, ...
                            9, 9, 'fdcc', 'hover', {'swing_half';'steady_0'}, ...
                            'ctc', 0, 1, 0, 0.6, deg2rad(30), ...
                            0, 0, 0);
default_config.is_static = 0;

% test_title = 'Actuation Cost during Freefall';
% test_name = './figures/traj_actcost_freefall';
% nc = 10;
% default_config.g = 0.001;
% default_config.g_vec = [0;0;default_config.g];

test_title = 'Actuation Cost during Hover';
test_name = './figures/traj_actcost_hover';
nc = 10;

% test_title = 'Actuation Cost during Translation';
% test_name = './figures/traj_actcost_trans';
% l_names = {'Hover Swing', 'Trans. Steady', 'Trans. Swing', 'Trans. Swing (\(t=\infty\))'};
% nc = 4;

configs(1:nc,1) = {default_config};

ts = round(logspace(0,1,nc-1),1);

% ts = 5*ones(1,nc);
% configs{1}.spline.tname_base = 'hover';
% configs{3}.spline.tname_r{1} = 'steady_0';

for i = 1:nc-1
    configs{i}.time.tf = ts(i);
end
configs{nc}.is_static = 1;


%% Simulation

results = cell(size(configs));
analyses = cell(size(configs));

%% Run Simulation
disp('--== Simulation ==--')
sn = state_names_lookup(configs{1}.model.n);

% for i = 1:size(configs,1)
parfor i = 1:size(configs,1)
    if i == 0
        is_static = 1;
    else
        is_static = 0;
    end
    
    disp(['Configuration ', num2str(i)])
    results{i} = mantis_traj_sim_simulate( configs{i}, camera );
    disp(' ')
end


%% Data Analysis
disp('--== Analysis ==--')

% for j = 1:size(configs,1)
parfor i = 1:size(configs,1)
    disp(['Configuration ', num2str(i)])
    analyses{i}.J_fuel = sum(abs(results{i}.c(sn.CONTROL_TAU,:)));
end
i = size(configs,1); % persist i after parfor


%% Plotting
brewermap('Spectral');
% brewermap('Dark2');
cmap = brewermap(size(configs,1));
c_names = cell(size(configs));

fJ = figure();
    clf;

    title(test_title)
    hold on;
    tmax = 0;
    tmin = inf;
    for i = 1:size(configs,1)
        plot(results{i}.t(2:end)/configs{i}.time.tf, analyses{i}.J_fuel(2:end), 'Color', cmap(i,:))
        
%         if configs{i}.time.tf < tmin
%             tmin = configs{i}.time.tf;
%         end
%         
%         if configs{i}.time.tf > tmax
%             tmax = configs{i}.time.tf;
%         end

        c_names{i} = num2str(configs{i}.time.tf,2);
        
%         if configs{i}.is_static
%             ldt = '\infty';
%         else
%             ldt = num2str(configs{i}.time.tf);
%         end
%         
%         l_names{i} = ['\(\Delta t=', ldt, '\)'];
    end
    c_names{end} = '\(\infty\)';

%     l = legend(l_names);
%     flushLegend(l,'Location','NorthWest');

    xticks([0,0.2,0.4,0.6,0.8,1])
    xticklabels({'0','20','40','60','80','100'})
    xlabel('Normalised Time (\%)')
    ylabel('Actuator Force (N)')
    
    colormap(brewermap);
    cb = colorbar('Ticks',linspace(0,1,size(configs,1)),...
         'TickLabels',c_names);
    cb.Label.String = 'Trajectory Time ($s$)';
    cb.Label.Interpreter = 'latex';
    set(cb,'TickLabelInterpreter','latex');


% %%
% a_ignore_map = [1,3,5,8];
% alnames = {'\(\omega_{bx}\)';'\(\omega_{by}\)';'\(\omega_{bz}\)'; ...
%            '\(v_{bx}\)';'\(v_{by}\)';'\(v_{bz}\)'; ...
%            '\(r_{1}\)';'\(r_{2}\)'};
% alnames(a_ignore_map) = [];
% 
% for i = 1:size(configs,1)
%     figure();
%     clf;
% 
%     title('Output Cost')
%     hold on;
%     tau = results{i}.c(sn.CONTROL_TAU,:);
%     tau(a_ignore_map,:) = [];
%     
%     acmap = lines(size(tau,1));
%     for j = 1:size(tau,1)
%         plot(results{i}.t(2:end)/configs{i}.time.tf, abs(tau(j,2:end)), 'Color', acmap(j,:))
%     end
%     l = legend(alnames,'Location','SouthWest');
% end
% 
%     print(fm, ['./figures/',test_name], '-depsc')
export_fig(test_name, '-eps', fJ)
export_fig(test_name, '-png', fJ)









