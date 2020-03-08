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


%%

t0 = 0;
tf = 10;

vp1 = sin(linspace(0,2*pi,5));
vp2 = sin(linspace(0,2*pi,7));
vp3 = sin(linspace(0,2*pi,9));
via_est_method = 'fdcc';

%%

num_via_points1 = length(vp1);
num_via_points2 = length(vp2);
num_via_points3 = length(vp3);

t_vias1 = linspace(t0,tf,num_via_points1);
t_vias2 = linspace(t0,tf,num_via_points2);
t_vias3 = linspace(t0,tf,num_via_points3);
sd1 = t_vias1(2) - t_vias1(1);
sd2 = t_vias2(2) - t_vias2(1);
sd3 = t_vias3(2) - t_vias3(1);

vias1 = gen_vias(via_est_method, vp1, sd1,'Conditioning','Unconditioned');
vias2 = gen_vias(via_est_method, vp1, sd1,'Conditioning','Precise');
vias3 = gen_vias(via_est_method, vp2, sd2,'Conditioning','Unconditioned');
vias4 = gen_vias(via_est_method, vp2, sd2,'Conditioning','Precise');
vias5 = gen_vias(via_est_method, vp3, sd3,'Conditioning','Unconditioned');
vias6 = gen_vias(via_est_method, vp3, sd3,'Conditioning','Precise');

%%
[t1, s1] = gen_spline(t0, tf, vias1, 5, 100);
[~, s2] = gen_spline(t0, tf, vias2, 5, 100);
[t3, s3] = gen_spline(t0, tf, vias3, 5, 100);
[~, s4] = gen_spline(t0, tf, vias4, 5, 100);
[t5, s5] = gen_spline(t0, tf, vias5, 5, 100);
[~, s6] = gen_spline(t0, tf, vias6, 5, 100);

%%

sin_ref1 = sin(linspace(0,2*pi,length(t1)));
sin_ref3 = sin(linspace(0,2*pi,length(t3)));
sin_ref5 = sin(linspace(0,2*pi,length(t5)));

s1_RMSE = sqrt(mean((sin_ref1 - s1(1,:)).^2));  % Root Mean Squared Error
s2_RMSE = sqrt(mean((sin_ref1 - s2(1,:)).^2));  % Root Mean Squared Error
s3_RMSE = sqrt(mean((sin_ref3 - s3(1,:)).^2));  % Root Mean Squared Error
s4_RMSE = sqrt(mean((sin_ref3 - s4(1,:)).^2));  % Root Mean Squared Error
s5_RMSE = sqrt(mean((sin_ref5 - s5(1,:)).^2));  % Root Mean Squared Error
s6_RMSE = sqrt(mean((sin_ref5 - s6(1,:)).^2));  % Root Mean Squared Error

fprintf('s1_RMSE\ts2_RMSE\ts3_RMSE\ts4_RMSE\ts5_RMSE\ts6_RMSE\n');
fprintf('%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t\n', s1_RMSE, s2_RMSE, s3_RMSE, s4_RMSE, s5_RMSE, s6_RMSE);

%%

f1 = figure();
    clf;
    subplot(1,3,1)
        hold on;
        plot(t1,sin_ref1, '--k');
        plot(t1,s1(1,:), '-b');
        plot(t2,s2(1,:), '-r');
        scatter(t_vias1, vp1, 'ro')
        hold off;
        
        grid on;
        title('\textbf{Piecewise Spline with 5 Vias}')
        xlabel('Time (\(s\))')
        ylim([-1.2,1.2])
        ylabel('Position (\(m\))')
        legend('Sine', 'Unconditioned','Precise','Vias')
        
    subplot(1,3,2)
        hold on;
        plot(t3,sin_ref3, '--k');
        plot(t3,s3(1,:), '-b');
        plot(t4,s4(1,:), '-r');
        scatter(t_vias2, vp2, 'ro')
        hold off;
        
        grid on;
        title('\textbf{Piecewise Spline with 7 Vias}')
        xlabel('Time (\(s\))')
        ylim([-1.2,1.2])
        ylabel('Position (\(m\))')
        legend('Sine', 'Unconditioned','Precise','Vias')
        
    subplot(1,3,3)
        hold on;
        plot(t5,sin_ref5, '--k');
        plot(t5,s5(1,:), '-b');
        plot(t6,s6(1,:), '-r');
        scatter(t_vias3, vp3, 'ro')
        hold off;
        
        grid on;
        title('\textbf{Piecewise Spline with 9 Vias}')
        xlabel('Time (\(s\))')
        ylim([-1.2,1.2])
        ylabel('Position (\(m\))')
        legend('Sine', 'Unconditioned','Precise','Vias')
        
    
%%
% print(f1, 'spline_bad_condition', '-depsc')
% print(f1, 'spline_bad_condition', '-dpng', '-r720')


