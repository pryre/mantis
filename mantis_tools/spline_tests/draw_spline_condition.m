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
dt = 1/500;
tf = 10;

vp = [0,-1,2,1,0,0];
via_est_method = 'fdcc';


%%
num_splines = 4;
num_via_points = length(vp);

t = t0:dt:tf;
t_vias = linspace(t0,tf,num_via_points);
sd = t_vias(2) - t_vias(1);

vias1 = gen_vias(via_est_method, vp, sd,'Conditioning','Unconditioned');
vias2 = gen_vias(via_est_method, vp, sd,'Conditioning','Holding');
vias3 = gen_vias(via_est_method, vp, sd,'Conditioning','Precise');
vias4 = gen_vias(via_est_method, vp, sd,'Conditioning','Exact');

vias(1,:,:) = vias1();
vias(2,:,:) = vias2;
vias(3,:,:) = vias3;
vias(4,:,:) = vias4;

via_steps = ((tf - t0) / dt) / (num_via_points - 1);
%%
[~, s1] = gen_spline(t0, tf, vias1, 5, via_steps);
[~, s2] = gen_spline(t0, tf, vias2, 5, via_steps);
[~, s3] = gen_spline(t0, tf, vias3, 5, via_steps);
[~, s4] = gen_spline(t0, tf, vias4, 5, via_steps);

%%
C = lines(4);

f1 = figure();
    clf;
    
%     via_steps = floor(length(t)/(length(v)-1));

for i = 1:length(vias1)-1
        ph = ((i-1)*via_steps)+1;
        pl = (i*via_steps);
        pr = ph:pl;

        order = 1;
        subplot(3,4,1)
            hold on;
            plot(t(pr),s1(order,pr), '-b');
            hold off;
        subplot(3,4,2)
            hold on;
            plot(t(pr),s2(order,pr), '-b');
            hold off;
        subplot(3,4,3)
            hold on;
            plot(t(pr),s3(order,pr), '-b');
            hold off;
        subplot(3,4,4)
            hold on;
            plot(t(pr),s4(order,pr), '-b');
            hold off;
            
        order = 2;
        subplot(3,4,5)
            hold on;
            plot(t(pr),s1(order,pr), '-b');
            hold off;
        subplot(3,4,6)
            hold on;
            plot(t(pr),s2(order,pr), '-b');
            hold off;
        subplot(3,4,7)
            hold on;
            plot(t(pr),s3(order,pr), '-b');
            hold off;
        subplot(3,4,8)
            hold on;
            plot(t(pr),s4(order,pr), '-b');
            hold off;
            
        order = 3;
        subplot(3,4,9)
            hold on;
            plot(t(pr),s1(order,pr), '-b');
            hold off;
        subplot(3,4,10)
            hold on;
            plot(t(pr),s2(order,pr), '-b');
            hold off;
        subplot(3,4,11)
            hold on;
            plot(t(pr),s3(order,pr), '-b');
            hold off;
        subplot(3,4,12)
            hold on;
            plot(t(pr),s4(order,pr), '-b');
            hold off;

end

for i = 1:3
    for j = 1:4
        subplot(3,4,(i-1)*4 + j)
            hold on;
            scatter(t_vias, vias(j,i,:), 'ro')
%             scatter(t_vias, vias2(i,:), 'ro')
%             scatter(t_vias, vias3(i,:), 'ro')
%             scatter(t_vias, vias4(i,:), 'ro')
            hold off;
            
            grid on;
            if i == 1
                ylim([-2.5,2.5])
            elseif i == 2
                ylim([-3,3])
            elseif i == 3
                xlabel('Time (\(s\))')
                ylim([-6.5,6.5])
            end
            
    end
end

subplot(3,4,1)
    title('\textbf{Unconditioned Quintic Spline}')
    ylabel('Position (\(m\))')

subplot(3,4,2)
    title('\textbf{Holding Quintic Spline}')
    
subplot(3,4,3)
    title('\textbf{Precise Quintic Spline}')
    
subplot(3,4,4)
    title('\textbf{Exact Quintic Spline}')
    
subplot(3,4,5)
    ylabel('Velocity (\(ms^{-1}\))')

subplot(3,4,9)
    ylabel('Accel. (\(ms^{-2}\))')
        
%%
print(f1, 'spline_condition', '-depsc')
print(f1, 'spline_condition', '-dpng', '-r720')


