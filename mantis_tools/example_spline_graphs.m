%% Setup

close all;
clear;
clc;


%% Parameters

t = 0:1:8;
x = [0, 0.5, 1, 1, 0.5, 0, 0, 0.5, 1];
y = [0, 0, 0, 1, 1, 1, 2, 2, 2];
%z = [0, 0, 0, 0, 0, 0];
tt = 0:0.1:8;

xx = spline(t,x,tt);
yy = spline(t,y,tt);
%zz = spline(t,z,tt);


%% Plot

f1 = figure(1);
    subplot(2,1,1)
        plot(t,x,'o',tt,xx)
        grid on;
        xlabel('Time (s)')
        ylabel('X Position (m)')
        title('Spline Interpolation vs. Time')
    subplot(2,1,2)
        plot(t,y,'o',tt,yy)
        grid on;
        xlabel('Time (s)')
        ylabel('Y Position (m)')
%     subplot(3,1,3)
%         plot(t,z,'o',tt,zz)
%         grid on;
%         xlabel('Time (s)')
%         ylabel('Z Position (m)')

f2 = figure(2);
    plot(x,y,'o',xx,yy)
    grid on;
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
    axis('square')
    title('Spacial Spline Trajectory')

    
print(f1, 'spline_vs_time.eps', '-depsc')
print(f2, 'spline_space.eps', '-depsc')
%%
print(f1, 'spline_vs_time.png', '-dpng')
print(f2, 'spline_space.png', '-dpng')