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
                            9, 9, 'fdcc', 'x_only_1m', {'swing_half';'steady_0'}, ...
                            'ctc', 0, 1, 0, 0.6, deg2rad(30), ...
                            0, 0, 0);
default_config.is_static = 0;

% test_title = 'Actuation Cost during Freefall';
% test_name = './figures/traj_actcost_freefall';
% nc = 10;
% default_config.g = 0.001;
% default_config.g_vec = [0;0;default_config.g];

% test_title = 'Actuation Cost during Hover';
% test_name = './figures/traj_actcost_hover';
% nc = 10;

test_title = 'Actuation Cost during Translation';
test_name = './figures/traj_actcost_trans';
l_names = {'Hover Swing', 'Trans. Steady', 'Trans. Swing', 'Trans. Swing (\(t=\infty\))'};
nc = 4;

configs(1:nc,1) = {default_config};

% ts = round(logspace(0,1,nc-1),1);

ts = 5*ones(1,nc);
configs{1}.spline.tname_base = 'hover';
configs{2}.spline.tname_r{1} = 'steady_0';

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

% for i = 1:size(configs,1)
parfor i = 1:size(configs,1)
    disp(['Configuration ', num2str(i)])
    analyses{i}.J_fuel = sum(abs(results{i}.c(sn.CONTROL_TAU,:)));
end
i = size(configs,1); % persist i after parfor


%% Plotting
% brewermap('Spectral');
brewermap('Dark2');
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

    l = legend(l_names);
    flushLegend(l,'Location','NorthWest');
    
%     colormap(brewermap);
%     cb = colorbar('Ticks',linspace(0,1,size(configs,1)),...
%          'TickLabels',c_names);
%     cb.Label.String = 'Trajectory Time ($s$)';
%     cb.Label.Interpreter = 'latex';
%     set(cb,'TickLabelInterpreter','latex');

    xticks([0,0.2,0.4,0.6,0.8,1])
    xticklabels({'0','20','40','60','80','100'})
    xlabel('Normalised Time (\%)')
    ylabel('Actuator Force (N)')

% export_fig(test_name, '-eps', fJ)
% export_fig(test_name, '-png', fJ)

%%
% Multirotor Parameters
mr = deg2rad(20);
fa = pi/6;
la = 0.25;

% Constraints
T_max = 0.8*9.80665; % From thrust tests
D_max = 1.2; % Lucky guess
% From XM430-W350 datasheet (selected as a "worst case" from performance graph)
rd1_max = 2.5;
rd2_max = 2.5;
r1_T_max = 2.5;
r2_T_max = 2.5;


% kTx = 0;
% kTy = 0;
% kTz = 1/(6*T_max);
% ktxi = 1/(sin(pi/6)*la*4*T_max); % Inner X-force calcs
% ktxo = 1/(la*2*T_max); % Outer X-force calcs
% kty = 1/(la*cos(pi/6)*4*T_max); % 
% ktz = 1/(6*D_max); % Outer X-force calcs

% Lateral Forces
kTxf = 1/(4*T_max*sin(mr)); % Forward X-force calcs
kTxr = 1/(2*T_max*sin(mr)); % Reverse X-force calcs
kTy = 1/(2*T_max*sin(mr));
kTz = 1/(6*T_max*cos(mr));

% Torquing Forces
kTtxi = 1/(sin(fa)*la*4*T_max*cos(mr)); % Inner X-force calcs
kTtxo = 1/(la*2*T_max*cos(mr)); % Outer X-force calcs
kTty = 1/(la*cos(fa)*4*T_max*cos(mr));
kTtz = 1/(la*6*T_max*sin(mr)); %Induced force from tilt

% Coriolis Forces
ktx = 0; %Induced torque from tilt (???)
kty = 0; %Induced torque from tilt (???)
ktz = 1/(6*D_max*cos(mr)) + kTtz; % Coriolis yaw force


Mb = [-kTtxo,   0, ktz, kTxr,   0, kTz; ...
       kTtxo,   0,-ktz, kTxr,   0, kTz; ...
       kTtxi,-kTy, ktz, kTxf,-kTy, kTz; ...
      -kTtxi, kTy,-ktz, kTxf, kTy, kTz; ...
      -kTtxi,-kTy,-ktz, kTxf, kTy, kTz; ...
       kTtxi, kTy, ktz, kTxf,-kTy, kTz];
   
Mr = [1/r1_T_max,0; ...
      0,1/r2_T_max];
  
M = [Mb, zeros(6,2);
     zeros(2,6), Mr]; 


%%
a_ignore_map = [1,3,5,8];
% alnames = {'\(\omega_{bx}\)';'\(\omega_{by}\)';'\(\omega_{bz}\)'; ...
%            '\(v_{bx}\)';'\(v_{by}\)';'\(v_{bz}\)'; ...
%            '\(r_{1}\)';'\(r_{2}\)'};
% alnames(a_ignore_map) = [];
alnames = {'\(m_{1}\)';'\(m_{2}\)';'\(m_{3}\)'; ...
           '\(m_{4}\)';'\(m_{5}\)';'\(m_{6}\)'; ...
           '\(r_{1}\)';'\(r_{2}\)'};


for i = 1:size(configs,1)
    fT = figure();
    clf;

    title(['Normalised Actuator Command -- ', l_names{i}])
    hold on;
    tau = results{i}.c(sn.CONTROL_TAU,:);
%     tau(a_ignore_map,:) = [];
    
    acmap = lines(size(tau,1));
    nm_val = zeros(8,size(tau,2));
    
    for j = 1:size(tau,2)
        nm_val(:,j) = M*tau(:,j);
    end
    
    for j = 1:size(nm_val,1)
        plot(results{i}.t(2:end)/configs{i}.time.tf, nm_val(j,2:end), 'Color', acmap(j,:))
    end
    
    ylim([0,1])
    xticks([0,0.2,0.4,0.6,0.8,1])
    xticklabels({'0','20','40','60','80','100'})
    xlabel('Normalised Time (\%)')
    yticks([0,0.2,0.4,0.6,0.8,1])
    yticklabels({'0','20','40','60','80','100'})
    ylabel('Actuator Command (\%)')
    
    l = legend(alnames);
    flushLegend(l,'Location','NorthWest');
    
    export_fig([test_name,'_throttle_case_', num2str(i)], '-eps', fT)
    export_fig([test_name,'_throttle_case_', num2str(i)], '-png', fT)
end








