%% Setup

close all;
clear;
clc;

set(0,'defaulttextInterpreter','latex');


%%

t0 = 0;
% dt = 1/125;
tf = 10;

vpx = [-2,-2,2,2];
vpy = [0,0.8,1.2,2];
via_est_method = 'fdcc';


%%
num_via_points = length(vpx);

% t = t0:dt:tf;
t_vias = linspace(t0,tf,num_via_points);
sd = t_vias(2) - t_vias(1);

viasxu = gen_vias(via_est_method, vpx, sd,'Conditioning','Unconditioned');
viasyu = gen_vias(via_est_method, vpy, sd,'Conditioning','Unconditioned');
viasxp = gen_vias(via_est_method, vpx, sd,'Conditioning','Precise');
viasyp = gen_vias(via_est_method, vpy, sd,'Conditioning','Precise');

% via_steps = ((tf - t0) / dt) / (num_via_points - 1);
%%
[t, sxu] = gen_spline(t0, tf, viasxu, 5, 200);
[~, syu] = gen_spline(t0, tf, viasyu, 5, 200);
[~, sxp] = gen_spline(t0, tf, viasxp, 5, 200);
[~, syp] = gen_spline(t0, tf, viasyp, 5, 200);

%%

f1 = figure();
    clf;
    subplot(2,2,1)
        hold on;
        plot(t,sxu(1,:), '-b');
        plot(t,sxp(1,:), '-r');
        scatter(t_vias, vpx, 'ro')
        hold off;
        
        grid on;
%         xlabel('Time (\(s\))')
%         ylim([-1.5,0.5])
        ylabel('X Position (\(m\))')
        title('\textbf{Linear Quintic Spline Trajectories}')
%         legend('Unconditioned','Precise','Vias')
        legend('Unconditioned','Precise', 'Vias', 'Location', 'SouthEast')
        
    subplot(2,2,3)
        hold on;
        plot(t,syu(1,:), '-b');
        plot(t,syp(1,:), '-r');
        scatter(t_vias, vpy, 'ro')
        hold off;
        
        grid on;
        xlabel('Time (\(s\))')
        ylim([-0.5,2.5])
        ylabel('Y Position (\(m\))')
%         legend('Unconditioned','Precise','Vias')
        legend('Unconditioned','Precise', 'Vias', 'Location', 'SouthEast')
 
    subplot(1,2,2)
        hold on;
        plot(sxu(1,:),syu(1,:), '-b');
        plot(sxp(1,:),syp(1,:), '-r');
        scatter(vpx(1,:), vpy(1,:), 'ro')
        %%
        obs_color = [0.4 0.4 0.4];
        rectangle('Position', [-1,0,5,0.8], 'FaceColor',obs_color, 'EdgeColor',obs_color);
        rectangle('Position', [-4,0,1,2], 'FaceColor',obs_color, 'EdgeColor',obs_color);
        rectangle('Position', [-3,1.2,4,0.8], 'FaceColor',obs_color, 'EdgeColor',obs_color);
        rectangle('Position', [3,0,1,2], 'FaceColor',obs_color, 'EdgeColor',obs_color);
        %%
        hold off;
        
        grid on;
        xlabel('Time (\(s\))')
        xlim([-4,4])
        ylabel('Position (\(m\))')
        legend('Unconditioned','Precise', 'Vias', 'Location', 'SouthEast')
        title('\textbf{2D Trajectory through Passage}')

        
%%
print(f1, 'spline_collision', '-depsc')
print(f1, 'spline_collision', '-dpng', '-r720')


