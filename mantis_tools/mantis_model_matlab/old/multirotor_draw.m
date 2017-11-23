function [outputArg1,outputArg2] = multirotor_draw( y )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here



    %y = [q1, qd1, q2, qd2, q3, qd3, q4, qd4, q5, qd5, q6, qd6]
    % params = [m1, 1.0; ...
    %           g, 9.80665; ...
    %           Ix1, (1/12)*1.0*(3*0.1^2 + 0.05^2); ...
    %           Iy1, (1/12)*1.0*(3*0.1^2 + 0.05^2); ...
    %           Iz1, 0.5*1.0*0.1^2];
        
    l = 0.25; % arm length
    
    dcm = angle2dcm(-y(11), -y(9), y(7));
    
    base = [y(1);y(3);y(5)]; 
    motor_1 = l*dcm*[sin(pi/4); -cos(pi/4); 0] + base;
    motor_2 = l*dcm*[-sin(pi/4); cos(pi/4); 0] + base;
    motor_3 = l*dcm*[sin(pi/4); cos(pi/4); 0] + base;
    motor_4 = l*dcm*[-sin(pi/4); -cos(pi/4); 0] + base;
    
    hold off;
    plot3([base(1),motor_1(1)], [base(2),motor_1(2)], [base(3),motor_1(3)], 'color', 'r', 'linewidth', 2);
    hold on;
    plot3([base(1),motor_2(1)], [base(2),motor_2(2)], [base(3),motor_2(3)], 'color', 'k', 'linewidth', 2);
    plot3([base(1),motor_3(1)], [base(2),motor_3(2)], [base(3),motor_3(3)], 'color', 'r', 'linewidth', 2);
    plot3([base(1),motor_4(1)], [base(2),motor_4(2)], [base(3),motor_4(3)], 'color', 'k', 'linewidth', 2);
   
    ax_s = 2;
    axis([-ax_s, ax_s, -ax_s, ax_s, 0, ax_s]);
    axis('square')
    
    drawnow;
end

