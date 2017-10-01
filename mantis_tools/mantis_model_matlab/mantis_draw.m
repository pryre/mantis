function [ ] = mantis_draw( y, p )
%MANTIS_DRAW renders the state of the Mantis MM-UAV
%   y: MM-UAV state matrix of form:
%      [ x; y; z; dx; dy; dz; ...
%        phi; theta; psi; dphi; dtheta; dpsi; ...
%        thetal1; thetal2; dthetal1; dthetal2 ]
%   p: Parameter structure as loaded by mantis_params()
       
    %% Draw reference frame
    hold off; % Clean the display area
    plot3([0,0.5], [0,0], [0,0], 'color', 'r', 'linewidth', 1);
    hold on;
    plot3([0,0], [0,0.5], [0,0], 'color', 'g', 'linewidth', 1);
    plot3([0,0], [0,0], [0,0.5], 'color', 'b', 'linewidth', 1);
    
    %% Draw multirotor
    
    % TODO: Account for rotation
    switch p.frame.layout
        case 'X6'
            fl = p.frame.motor_arm_length;
            x30 = fl*cos(deg2rad(30));
            y30 = fl*sin(deg2rad(30));
            
            plot3([y(1),y(1)+x30], [y(2),y(2)+y30], [y(3),y(3)], 'color', 'r', 'linewidth', 2);
            plot3([y(1),y(1)+x30], [y(2),y(2)-y30], [y(3),y(3)], 'color', 'r', 'linewidth', 2);
            plot3([y(1)-x30,y(1)], [y(2)-y30,y(2)], [y(3),y(3)], 'color', 'k', 'linewidth', 2);
            plot3([y(1)-x30,y(1)], [y(2)+y30,y(2)], [y(3),y(3)], 'color', 'k', 'linewidth', 2);
            plot3([y(1),y(1)], [y(2)-fl,y(2)+fl], [y(3),y(3)], 'color', 'k', 'linewidth', 2);
        otherwise
            error('Unsupported motor layout')
    end
    
    %% Draw arm
    al = p.arm.length;
    amp = p.arm.mount.point;
    l0rot = p.arm.mount.rotation;
    l0x = y(1) + amp.x; % TODO: Account for rotation
    l0y = y(2) + amp.y;
    l0z = y(3) + amp.z;
    
    l1rot = l0rot + y(13);
    l1x = l0x + cos(l1rot)*al;
    l1y = l0y; % TODO: Account for rotation
    l1z = l0z + sin(l1rot)*al;
    plot3([l0x,l1x], [l0y,l1y], [l0z,l1z], 'color', 'g', 'linewidth', 2);
    
    l2rot = l1rot + y(14);
    l2x = l1x + cos(l2rot)*al;
    l2y = l1y; % TODO: Account for rotation
    l2z = l1z + sin(l2rot)*al;
    plot3([l1x,l2x], [l1y,l2y], [l1z,l2z], 'color', 'g', 'linewidth', 2);
    
    disp(['End Effector: [', num2str(l2x), ',', num2str(l2y), ',', num2str(l2z), ']']);
    
    
    %% Draw
    ax_s = p.plot.size / 2;
    
    axis([-ax_s, ax_s, -ax_s, ax_s, 0, 2*ax_s]);
    axis('square')
    
    drawnow;
    
end