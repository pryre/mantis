%% Setup
close all;
clear;
clc;


%% Variables

% Body Links
% When defining the body links, the first link should be the base_link
% All other links should be defined relative to this frame of reference
% i.e. the base link should represent position [0,0,0], and rotation
% [0,0,0,1] of the body
% Link list layout:
%           { name, type, mass, center_of_mass:[x;y;z], frame_rotation:[w,x,y,z], dimensions:[...]}
body = {};
body = add_body_link(body, 'base_link', 'cylinder', 0.38, [0;0;0], angle2quat(0,0,0), [0.105,0.03]);
body = add_body_link(body, 'battery', 'cuboid', 0.58, [0;0;0.032], angle2quat(0,0,0), [0.052,0.146,0.034]);
body = add_body_link(body, 'arm_1', 'cylinder', 0.09, [0;-0.19;0], angle2quat(0,0,pi/2), [0.015,0.17]);
body = add_body_link(body, 'arm_2', 'cylinder', 0.09, [0;0.19;0], angle2quat(0,0,pi/2), [0.015,0.17]);
body = add_body_link(body, 'arm_3', 'cylinder', 0.09, [0.165;0.095;0], angle2quat(2*pi/3,0,pi/2), [0.015,0.17]);
body = add_body_link(body, 'arm_4', 'cylinder', 0.09, [-0.165;-0.095;0], angle2quat(2*pi/3,0,pi/2), [0.015,0.17]);
body = add_body_link(body, 'arm_5', 'cylinder', 0.09, [0.165;-0.095;0], angle2quat(pi/3,0,pi/2), [0.015,0.17]);
body = add_body_link(body, 'arm_6', 'cylinder', 0.09, [-0.165;0.095;0], angle2quat(pi/3,0,pi/2), [0.015,0.17]);
body = add_body_link(body, 'skid_1', 'cylinder_h', 0.008, [0;-0.275;-0.075], angle2quat(0,0,0), [0.007,0.008,0.12]);
body = add_body_link(body, 'skid_2', 'cylinder_h', 0.008, [0;0.275;-0.075], angle2quat(0,0,0), [0.007,0.008,0.12]);
body = add_body_link(body, 'skid_3', 'cylinder_h', 0.008, [0.2382;0.1375;-0.075], angle2quat(0,0,0), [0.007,0.008,0.12]);
body = add_body_link(body, 'skid_4', 'cylinder_h', 0.008, [-0.2382;-0.1375;-0.075], angle2quat(0,0,0), [0.007,0.008,0.12]);
body = add_body_link(body, 'skid_5', 'cylinder_h', 0.008, [0.2382;-0.1375;-0.075], angle2quat(0,0,0), [0.007,0.008,0.12]);
body = add_body_link(body, 'skid_6', 'cylinder_h', 0.008, [-0.2382;0.1375;-0.075], angle2quat(0,0,0), [0.007,0.008,0.12]);

%% Calc
body_inertial = zeros(3,3);
body_com = zeros(1,3);  % Center of Mass
body_mass = 0;

for i = 1:size(body, 1)
    link_inertial = zeros(3,3);
    link_name = body{i,1};
    link_type = body{i,2};
    link_mass = body{i,3};
    link_com = body{i,4};
    link_rot = body{i,5};
    link_dim = body{i,6};
  
    switch link_type
        case 'cuboid'
            link_inertial = calc_cuboid(link_mass, link_dim);
        case 'cylinder'
            link_inertial = calc_cylinder(link_mass, link_dim);
        case 'cylinder_h'
            link_inertial = calc_cylinder_h(link_mass, link_dim);
        case 'sphere'
            link_inertial = calc_sphere(link_mass, link_dim);
        case 'sphere_h'
            link_inertial = calc_sphere_h(link_mass, link_dim);
        case 'ellipsoid'
            link_inertial = calc_ellipsoid(link_mass, link_dim);
        otherwise
            disp(['Geometry (', link_type, ') is not supported for part ', link_name])
    end

    body(i,7) = {link_inertial};
    
    % Add the next part to the body inertial
    if body_mass ~= 0
        % Calculate new CoM location
        d_mass = link_mass / (link_mass + body_mass);
        d_position = d_mass * (body_com - link_com);
        new_body_com = body_com - d_position;
        
        % Rotate next link_inertial into the body frame
        Rbl = quat2rotm(link_rot);
        link_inertial_b = (Rbl*link_inertial)*(Rbl');
        
        % Calculate inertial offsets for each axis of both bodys
        d_link_com = new_body_com - link_com;
        d_body_com = new_body_com - body_com;
        
        new_link_inertial_b = link_inertial_b;
        new_body_inertial = body_inertial;
        
        % Use parallel axis theorem
        new_link_inertial_b(1,1) = link_inertial_b(1,1) + link_mass*((d_link_com(2)^2)+(d_link_com(3)^2));
        new_link_inertial_b(2,2) = link_inertial_b(2,2) + link_mass*((d_link_com(1)^2)+(d_link_com(3)^2));
        new_link_inertial_b(3,3) = link_inertial_b(3,3) + link_mass*((d_link_com(1)^2)+(d_link_com(2)^2));
        
        new_body_inertial(1,1) = body_inertial(1,1) + body_mass*((d_body_com(2)^2)+(d_body_com(3)^2));
        new_body_inertial(2,2) = body_inertial(2,2) + body_mass*((d_body_com(1)^2)+(d_body_com(3)^2));
        new_body_inertial(3,3) = body_inertial(3,3) + body_mass*((d_body_com(1)^2)+(d_body_com(2)^2));
        
        % Calculate new body mass
        body_com = new_body_com;
        body_mass = body_mass + link_mass;
        body_inertial = new_link_inertial_b + new_body_inertial;
    else
        % This is the first pass, so use this part as the base link
        body_com = link_com;
        body_mass = link_mass;
        body_inertial = link_inertial;
    end
end

% TODO: Calculate Inertial around base link again

%% Results
disp('Body Mass:')
disp(body_mass)
disp('Body CoM:')
disp(body_com)
disp('Body Inertial:')
disp(body_inertial)
