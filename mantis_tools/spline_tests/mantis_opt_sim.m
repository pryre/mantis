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


test_name = './figures/traj_optcost';
% l_names = {'Hover Swing', 'Trans. Steady', 'Trans. Swing', 'Trans. Swing (\(t=\infty\))'};

optimise_path_step = 0.5;
optimise_seg_step = 0.5;

kJ = 0.8;

e = 0.005; % 1% of the maximum actuator command
e_step = 0.0001;

num_samples = 100;

nv = 5;
vt = 50*[0,1,2,3,4];
vpx = [0, 1, 1.5, 2.5,  7];
vpy = [0, 2,  -1,   0, -1];
vpz = [0, 1,  0,   1, 2];
vppsi = [pi/2, 0,  pi/4,   0, -pi/4];

dt = (vt(2)-vt(1))/num_samples;

config = gen_config(vt(1), dt, dt, vt(end), ...
                    'quad_x4', 'serial', 2, ...
                    9, nv, 'fdcc', 'hover', {'swing_half';'steady_0'}, ...
                    'ctc', 0, 1, 0, 0.6, deg2rad(30), ...
                    0, 0, 0);

vpr = cell(config.model.n,1);
for i = 1:config.model.n
    vpr{i} = gen_trajectory_vias_r(config.spline.tname_r{i}, config.spline.num_vias);
end


%% Motor Mapping
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


results = cell(0,1);
analyses = cell(0,1);
sn = state_names_lookup(config.model.n);


%% Optimisation Step 1 - Whole path performance
% Necessary to do first to optimise the derivatives
disp('Path Duration')
step = optimise_path_step;
vt_last = vt;

i = 1;
num_optim = 0;
while true    
    % Shrink time scale such that as the step decreases, changes also
    % decrease
    vias.time = (1-step)*vt_last;
    
%     t = vias.time(1):config.time.dt:vias.time(end);
    % vias.time = t(floor(linspace(1,length(t),config.spline.num_vias)));

    % [vpx, vpy, vpz, vppsi] = gen_trajectory_vias(config.spline.tname_base, config.spline.num_vias);
    % vpr = cell(config.model.n,1);
    % r0 = zeros(config.model.n,1);
    % for i = 1:config.model.n
    %     vpr{i} = gen_trajectory_vias_r(config.spline.tname_r{i}, config.spline.num_vias);
    %     r0(i) = vpr{i}(1); % Easier to capture initial joint states in this loop
    % end

    vias.x = gen_vias(config.spline.dvia_est_method, vpx, vias.time);
    vias.y = gen_vias(config.spline.dvia_est_method, vpy, vias.time);
    vias.z = gen_vias(config.spline.dvia_est_method, vpz, vias.time);
    vias.psi = gen_vias(config.spline.dvia_est_method, vppsi, vias.time);
    vias.r = cell(config.model.n,1);
    for j = 1:config.model.n
        vias.r{j} = gen_vias(config.spline.dvia_est_method, vpr{j}, vias.time);
    end


    %% Run Simulation
    result = mantis_opt_sim_simulate( config, camera, vias, num_samples);
    result.vias = vias;
    
    %% Data Analysis
    tau = result.c(sn.CONTROL_TAU,:);
    analysis.J_fuel = sum(abs(tau));
    
    analysis.J_act = zeros(8,size(tau,2));

    for j = 1:size(tau,2)
        analysis.J_act(:,j) = M*tau(:,j);
    end
    
    max_J_act = max(max(analysis.J_act));
    
    J1 = kJ - max_J_act;
    
    if J1 > 0
        % Record improved values 
        num_optim = num_optim + 1;
        
        results{num_optim} = result;
        analyses{num_optim} = analysis;
    end
    
    if ( (J1 < e) && (J1 > 0) ) || (step < e_step )
        disp('Well optimised!')
        if (step < e_step )
            disp('Cannot optimise further')
        end
        break
    elseif J1 > 0
        disp('shrink!')
        vt_last = vias.time;
    elseif J1 < 0
        disp('decrease step...')
        step = optimise_path_step*step;
    end
    
    
    i = i + 1;
end
disp(' ')

num_optim_step_1 = num_optim;
vt_last_step_1 = vt_last;


%% Optimisation Step 2 - Segment aggression
% Forces each segment to perform to the same standard
vias_whole_path = vias;

i = 1;
for k = 2:size(vt,2)
    disp(['Segment ', num2str(k-1)])
    step = optimise_seg_step;
    
    while true
        
        % Keep via values static
%         vias = vias_whole_path;

        % Shrink time scale such that as the step decreases, changes also
        % decrease
        vias.time = vt_last;
        dvt = vias.time(k) - vias.time(k-1);
        dvt_new = (1-step)*dvt;
        dvt_diff = dvt - dvt_new;
        vias.time(k:end) = vias.time(k:end) - dvt_diff;
        
    %%
    %     t = vias.time(1):config.time.dt:vias.time(end);
        % vias.time = t(floor(linspace(1,length(t),config.spline.num_vias)));

        % [vpx, vpy, vpz, vppsi] = gen_trajectory_vias(config.spline.tname_base, config.spline.num_vias);
        % vpr = cell(config.model.n,1);
        % r0 = zeros(config.model.n,1);
        % for i = 1:config.model.n
        %     vpr{i} = gen_trajectory_vias_r(config.spline.tname_r{i}, config.spline.num_vias);
        %     r0(i) = vpr{i}(1); % Easier to capture initial joint states in this loop
        % end

%         vias.x = gen_vias(config.spline.dvia_est_method, vpx, vias.time);
%         vias.y = gen_vias(config.spline.dvia_est_method, vpy, vias.time);
%         vias.z = gen_vias(config.spline.dvia_est_method, vpz, vias.time);
%         vias.psi = gen_vias(config.spline.dvia_est_method, vppsi, vias.time);
%         vias.r = cell(config.model.n,1);
%         for j = 1:config.model.n
%             vias.r{j} = gen_vias(config.spline.dvia_est_method, vpr{j}, vias.time);
%         end


        %% Run Simulation
        result = mantis_opt_sim_simulate( config, camera, vias, num_samples);
        result.vias = vias;
        
        
        %% Data Analysis
        tau = result.c(sn.CONTROL_TAU,:);
        analysis.J_fuel = sum(abs(tau));

        analysis.J_act = zeros(8,size(tau,2));

        for j = 1:size(tau,2)
            analysis.J_act(:,j) = M*tau(:,j);
        end

        % Only look to optimise the current segment
        inds0 = find(result.t==vias.time(k-1));
        indsf = find(result.t==vias.time(k));
        max_J_act = max(max(analysis.J_act(:,inds0:indsf)));

        J1 = kJ - max_J_act;

        if J1 > 0
            % Record improved values 
            num_optim = num_optim + 1;

            results{num_optim} = result;
            analyses{num_optim} = analysis;
        end

        if ( (J1 < e) && (J1 > 0) ) || (step < e_step )
            disp('Well optimised!')
            if (step < e_step )
                disp('Cannot optimise further')
            end
            break
        elseif J1 > 0
            disp('shrink!')
            vt_last = vias.time;
        elseif J1 < 0
            disp('decrease step...')
            step = optimise_path_step*step;
        end


        i = i + 1;
    end
    
    disp(' ')
end




%%


if num_optim <= 0
    error('No solution found')    
end


%% Plotting
brewermap('Spectral');
cmap = brewermap(num_optim+1);

ft = figure();
    clf;

    title('Optimised 3D Trajectory')
    hold on;
    tmax = 0;
    tmin = inf;
    scatter3(vias.x(1,:),vias.y(1,:),vias.z(1,:), 'or')
    
    for i = 1:num_optim
        plot3(results{i}.traj.s_x(1,:),results{i}.traj.s_y(1,:),results{i}.traj.s_z(1,:),'Color',cmap(i,:));
    end
    
    axis('equal')
    xlabel('X Position ($m$)')
    ylabel('Y Position ($m$)')
    zlabel('Z Position ($m$)')
    view([-45,35])
    set(gca,'SortMethod','ChildOrder')

fig_name = [test_name, '_traj'];
export_fig(fig_name, '-eps', ft)
export_fig(fig_name, '-png', ft)


%%
a_ignore_map = [1,3,5,8];
% alnames = {'\(\omega_{bx}\)';'\(\omega_{by}\)';'\(\omega_{bz}\)'; ...
%            '\(v_{bx}\)';'\(v_{by}\)';'\(v_{bz}\)'; ...
%            '\(r_{1}\)';'\(r_{2}\)'};
% alnames(a_ignore_map) = [];
alnames = {'$k_{J}$';  '$t_{\mathcal{Q}}$'
           '\(m_{1}\)';'\(m_{2}\)';'\(m_{3}\)'; ...
           '\(m_{4}\)';'\(m_{5}\)';'\(m_{6}\)'; ...
           '\(r_{1}\)';'\(r_{2}\)'};

acmap = brewermap(8);

for i = [1,num_optim_step_1,num_optim]
    fT = figure();
    clf;

    step_t_lim = [results{i}.vias.time(1),results{i}.vias.time(end)];
    
    title(['Normalised Actuator Command -- Step ', num2str(i)])
    hold on;
    % Plot actuator limit and via segemnts
    plot(step_t_lim, [kJ,kJ], '--k')
    for j = 1:size(results{i}.vias.time,2)
        plot([results{i}.vias.time(j),results{i}.vias.time(j)], [0,1], '--r')
    end
    
%     tau(a_ignore_map,:) = [];
    
    for j = 1:size(analyses{i}.J_act,1)
        plot(results{i}.t, analyses{i}.J_act(j,:), 'Color', acmap(j,:))
    end
    
    xlim(step_t_lim)
    ylim([0,1])
%     xticks([0,0.2,0.4,0.6,0.8,1])
%     xticklabels({'0','20','40','60','80','100'})
    xlabel('Time ($s$)')
    yticks([0,0.2,0.4,0.6,0.8,1])
    yticklabels({'0','20','40','60','80','100'})
    ylabel('Actuator Command (\%)')
    
    if i == 1
        % Fake data for legend
        lp = [];
        lp(end+1) = plot(nan, nan, '--k');
        lp(end+1) = plot(nan, nan, '--r');
        for j = 1:size(analyses{i}.J_act,1)
            lp(end+1) = plot(nan, nan, 'Color', acmap(j,:));
        end
        
        l = legend(lp,alnames);
        flushLegend(l,'Location','NorthWest');
    end
    fig_name = [test_name, '_step_', num2str(i)];
    export_fig(fig_name, '-eps', fT)
    export_fig(fig_name, '-png', fT)
end








