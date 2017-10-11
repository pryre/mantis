function [ ] = arm_draw( y )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    %y = [q1, qd1, q2, qd2]
    %Params
    %  l1, 0.25; ...
    %  l2, 0.25; ...
    %  lc1, 0.25/2; ...
    %  lc2, 0.25/2; ...
    %  g, 9.80665; ...
    %  m1, 0.2; ...
    %  m2, 0.2; ...
    %  Iz1, 0.2*(0.25^2)/12; ...
    %  Iz2, 0.2*(0.25^2)/12];
    
    l1 = 0.25;
    l2 = 0.25;
    
    l1_x = l1*cos(y(1));
    l1_y = l1*sin(y(1));
    l2_x = l2*cos(y(1)+y(3)) + l1_x;
    l2_y = l2*sin(y(1)+y(3)) + l1_y;
    
    hold off;
    plot([0,l1_x], [0,l1_y], 'color', 'k', 'linewidth', 2);
    hold on;
    plot([l1_x,l2_x], [l1_y,l2_y], 'color', 'k', 'linewidth', 2);

    ax_s = 2*(l1 + l2);
    
    axis([-ax_s, ax_s, -ax_s, ax_s, 0, 2*ax_s]);
    axis('square')
    
    drawnow;
end

