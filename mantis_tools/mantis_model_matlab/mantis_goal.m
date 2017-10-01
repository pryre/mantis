function [ yf ] = mantis_goal( base_pos, arm_pos, p )
%MANTIS_GOAL Calculates the final state goal for some input goals
%   base: Goal position of the base link as [X,Y,Z]
%   arm: Goal position of the end effector as [X,Y,Z]
%   p: Parameter structure as loaded by mantis_params()
%
%   yf: MM-UAV final state matrix of form:
%       [ x; y; z; dx; dy; dz; ...
%         phi; theta; psi; dphi; dtheta; dpsi; ...
%         thetal1; thetal2; dthetal1; dthetal2 ]


    %% Generate the arm
    
    % Mount servo
    arm.link0.len = 0;
    arm.link0.enc = 0;
    arm.link0.limP = p.arm.servo.max_rot;
    arm.link0.limN = -p.arm.servo.max_rot;
    arm.link0.ang = p.arm.mount.rotation;
    arm.link0.end.x = p.arm.mount.point.x;
    arm.link0.end.y = p.arm.mount.point.z;

    % Upper arm
    arm.link1.len = p.arm.length;
    arm.link1.enc = 0;
    arm.link1.limP = p.arm.servo.max_rot;
    arm.link1.limN = -p.arm.servo.max_rot;
    arm.link1.ang = 0;
    arm.link1.end.x = 0;
    arm.link1.end.y = 0;

    % Forearm
    arm.link2.len = p.arm.length;
    arm.link2.enc = 0;
    arm.link2.limP = p.arm.servo.max_rot;
    arm.link2.limN = -p.arm.servo.max_rot;
    arm.link2.ang = 0;
    arm.link2.end.x = 0;
    arm.link2.end.y = 0;

    names = fieldnames(arm);
    numLinks = size(names,1);

    for i = 1:numLinks-1
        arm.(names{i+1}).ang = arm.(names{i}).enc + arm.(names{i}).ang;
        arm.(names{i+1}).end.x = arm.(names{i}).end.x + arm.(names{i+1}).len*cos(arm.(names{i+1}).ang);
        arm.(names{i+1}).end.y = arm.(names{i}).end.y + arm.(names{i+1}).len*sin(arm.(names{i+1}).ang);
    end
    
    
    %% Calculates joint angles
   
    pos_diff = arm_pos - base_pos;
    
    % From here on, only work in the 2D frame
    %   xt is forwards-backwards
    %   yt is up-down
    
    psi = atan2(pos_diff(2), pos_diff(1));
    len = sqrt(pos_diff(1)^2 + pos_diff(2)^2);
    
    % World targets
    xyt = len - arm.link0.end.x;
    zt = pos_diff(3) - arm.link0.end.y;
    
    % Local targets
    a = -arm.link0.ang;
    p = [xyt,zt]*[cos(a),sin(a);-sin(a),cos(a)];
    xl = p(1);
    yl = p(2);
    
    % Do joint math
    % This will flip xl and yl to have the correct bend
    c2 = (xl^2 + yl^2 - arm.link1.len^2 - arm.link2.len^2)/(2*arm.link1.len*arm.link2.len);
    s2 = sqrt(1 - c2^2);
    theta2 = atan2(s2, c2); % theta2 is deduced

    k1 = arm.link1.len + arm.link2.len*c2;
    k2 = arm.link2.len*s2;
    theta1 = atan2(yl, xl) - atan2(k2, k1); % theta1 is deduced

    if (theta2>arm.link1.limP)||(theta2<arm.link1.limN)||(theta1>arm.link0.limP)||(theta1<arm.link0.limN)
        error('Point outside of reachable area');
    end

    %disp(['Using [', num2str(theta1), ',', num2str(theta2), ...
    %      '] to reach point [', num2str(xl), ',', num2str(yl), ']']);

    
    %% Generate final state goal
    
    yf = [ base_pos(1); base_pos(2); base_pos(3); 0; 0; 0; ...
           0; 0; psi; 0; 0; 0; ...
           theta1; theta2; 0; 0];
       
    %disp('Goal state:');
    %disp(yf);

end

