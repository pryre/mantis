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
    
    mr_pos = [y(1); y(2); y(3)];
    mr_rot = angle2dcm(-y(9),-y(8),-y(7));
    
    switch p.frame.layout
        case 'X6'
            fl = p.frame.motor_arm_length;
            x30 = fl*cos(deg2rad(30));
            y30 = fl*sin(deg2rad(30));
            
            %frame arm 1 a-b
            fa0 = [0; 0; 0];
            fa1 = [0; -fl; 0];
            fa2 = [0; +fl; 0];
            fa3 = [x30; y30; 0];
            fa4 = [-x30; -y30; 0];
            fa5 = [x30; -y30; 0];
            fa6 = [-x30; +y30; 0];
            
            fa0 = fa0 + mr_pos;
            fa1 = (mr_rot * fa1) + mr_pos;
            fa2 = (mr_rot * fa2) + mr_pos;
            fa3 = (mr_rot * fa3) + mr_pos;
            fa4 = (mr_rot * fa4) + mr_pos;
            fa5 = (mr_rot * fa5) + mr_pos;
            fa6 = (mr_rot * fa6) + mr_pos;
            
            plot3([fa0(1),fa1(1)], [fa0(2),fa1(2)], [fa0(3),fa1(3)], 'color', 'k', 'linewidth', 2);
            plot3([fa0(1),fa2(1)], [fa0(2),fa2(2)], [fa0(3),fa2(3)], 'color', 'k', 'linewidth', 2);
            plot3([fa0(1),fa3(1)], [fa0(2),fa3(2)], [fa0(3),fa3(3)], 'color', 'r', 'linewidth', 2);
            plot3([fa0(1),fa4(1)], [fa0(2),fa4(2)], [fa0(3),fa4(3)], 'color', 'k', 'linewidth', 2);
            plot3([fa0(1),fa5(1)], [fa0(2),fa5(2)], [fa0(3),fa5(3)], 'color', 'r', 'linewidth', 2);
            plot3([fa0(1),fa6(1)], [fa0(2),fa6(2)], [fa0(3),fa6(3)], 'color', 'k', 'linewidth', 2);
        otherwise
            error('Unsupported motor layout')
    end
    
    
    %% Draw arm
    
    al = p.arm.length;
    amp = p.arm.mount.point;
    l0rot = p.arm.mount.rotation;
    l0 = [amp.x; amp.y; amp.z];
    
    % Calculate first link end point
    l1rot = l0rot + y(13);
    l1 = [l0(1) + cos(l1rot)*al; l0(2); l0(3) + sin(l1rot)*al];
    
    % Calculate second link end point
    l2rot = l1rot + y(14);
    l2 = [l1(1) + cos(l2rot)*al; l1(2); l1(3) + sin(l2rot)*al];
    
    l0 = (mr_rot * l0) + mr_pos;
    l1 = (mr_rot * l1) + mr_pos;
    l2 = (mr_rot * l2) + mr_pos;
    
    plot3([l0(1),l1(1)], [l0(2),l1(2)], [l0(3),l1(3)], 'color', 'g', 'linewidth', 2);
    plot3([l1(1),l2(1)], [l1(2),l2(2)], [l1(3),l2(3)], 'color', 'g', 'linewidth', 2);
    
    %disp(['End Effector: [', num2str(l2(1)), ',', num2str(l2(2)), ',', num2str(l2(3)), ']']);
    
    
    %% Draw
    
    ax_s = p.plot.size / 2;
    
    axis([-ax_s, ax_s, -ax_s, ax_s, 0, 2*ax_s]);
    axis('square')
    
    drawnow;
    
    
end