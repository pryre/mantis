%% Setup

close all;
clear;
clc;

set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesXGrid','on');
set(0,'DefaultAxesYGrid','on');
set(0,'DefaultAxesZGrid','on');
% set(0,'DefaultAxesSortMethod','ChildOrder');
set(0,'DefaultFigureColor','w');
% set(0, 'DefaultFigureRenderer', 'painter');


%%

t0 = 0;
dt = 1/200;
tf = 9;

vpx = [-1,-1,1,1];
% vpy = [0,0.8,1.2,2];
vpy = [0,1.5,2.5,4];
via_est_method = 'fdcc';


%%
num_via_points = length(vpx);

t_vias = linspace(t0,tf,num_via_points);

viasxu = gen_vias(via_est_method, vpx, t_vias,'Conditioning','Unconditioned');
viasyu = gen_vias(via_est_method, vpy, t_vias,'Conditioning','Unconditioned');
viasxp = gen_vias(via_est_method, vpx, t_vias,'Conditioning','Precise');
viasyp = gen_vias(via_est_method, vpy, t_vias,'Conditioning','Precise');

% via_steps = ((tf - t0) / dt) / (num_via_points - 1);
%%
[t, sxu] = gen_spline(t_vias, viasxu, 5, dt);
[~, syu] = gen_spline(t_vias, viasyu, 5, dt);
[~, sxp] = gen_spline(t_vias, viasxp, 5, dt);
[~, syp] = gen_spline(t_vias, viasyp, 5, dt);

%%

f1 = figure();
    clf;
    subplot(2,1,1)
        hold on;
        plot(t,sxu(1,:), '-b');
        plot(t,sxp(1,:), '-r');
        scatter(t_vias, vpx, 'ro')
        hold off;
        
%         xlabel('Time (\(s\))')
%         ylim([-1.5,0.5])
        ylabel('X Position (\(m\))')
        title('\textbf{Linear Quintic Spline Trajectories}')
%         legend('Unconditioned','Precise','Vias')
        legend('Unconditioned','Precise', 'Vias', 'Location', 'SouthEast')
        
    subplot(2,1,2)
        hold on;
        plot(t,syu(1,:), '-b');
        plot(t,syp(1,:), '-r');
        scatter(t_vias, vpy, 'ro')
        hold off;
        
        xlabel('Time (\(s\))')
        ylim([-2,6])
        ylabel('Y Position (\(m\))')
%         legend('Unconditioned','Precise','Vias')
        legend('Unconditioned','Precise', 'Vias', 'Location', 'SouthEast')
 %%
f2 = figure();
    hold on;
    plot(sxu(1,:),syu(1,:), '-b');
    plot(sxp(1,:),syp(1,:), '-r');
    scatter(vpx(1,:), vpy(1,:), 'ro')
    
    obs_color = [0.4 0.4 0.4];
    rectangle('Position', [-0.5,0,2.5,1.5], 'FaceColor',obs_color, 'EdgeColor',obs_color);
    rectangle('Position', [-2,0,0.5,2.5], 'FaceColor',obs_color, 'EdgeColor',obs_color);
    rectangle('Position', [-2,2.5,2.5,1.5], 'FaceColor',obs_color, 'EdgeColor',obs_color);
    rectangle('Position', [1.5,1.5,0.5,2.5], 'FaceColor',obs_color, 'EdgeColor',obs_color);
    
    hold off;

    axis('square')
    axis([-2 2 0 4])
    xlabel('X Position (\(m\))')
    ylabel('Y Position (\(m\))')
    legend('Unconditioned','Precise', 'Vias', 'Location', 'SouthEast')
    title('\textbf{2D Trajectory through Passage}')

        
%%
export_fig('./figures/spline_collision_axes', '-eps', f1)
export_fig('./figures/spline_collision_2d', '-eps', f2)


