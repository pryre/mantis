% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ p ] = mantis_params( filepath )
%MANTIS_PARAMS Generates the parameter structure for simulating a MM-UAV
%   filepath: path to the parameter file

    p = YAML.read(filepath);

    % The frame layout is used to determine how much of the available
    % thrust can be used to either act as thrust, or to cause a torque
    % effect on the base_link itself.
    %
    % The use of the thrust_reserve allows for different scaling of the max
    % thrust and torque control outputs. Setting the reserve to 25% will
    % ensure that the base link will always be able to output at least 25%
    % thrust for torque, with the remaining 75% thrust being avaiable for
    % actual thrust vectoring output.
    switch p.frame.layout
        case 'X4'
            disp('Using PixHawk X4 motor layout')
            
            % 3     1
            %  \   /
            %    x
            %  /   \
            % 2     4
            
            arm_ang = pi/4;
            
            p.frame.map = [  -arm_ang; ...
                            3*arm_ang; ...
                              arm_ang; ...
                           -3*arm_ang];
                       
            nm = numel(p.frame.map);
            
            thrust_scale = p.control.base_link.thrust.reserve;
            la = p.frame.motor_radius;
            max_thrust = p.motor.max_thrust;
            p.motor.num = nm;
            
            t_x = thrust_scale * max_thrust * ( ...
                      2 * ( cos(arm_ang) * la  ) ...
                  );
              
            t_y = thrust_scale * max_thrust * ( ...
                      2 * ( sin(arm_ang) * la ) ...
                  );

            p.control.base_link.torque.x_max = t_x;
            p.control.base_link.torque.y_max = t_y;
            
            p.control.base_link.thrust.max = nm * max_thrust * ...
                                             ( 1.0 - p.control.base_link.thrust.reserve );
            p.control.base_link.thrust.min = 0; % This could be potentially used for negative thrust platforms
            
            kT = 1/(nm*max_thrust);
            kt = 1/(4*la*cos(arm_ang)*max_thrust);
            km = 1/(nm*p.motor.prop_drag);

            p.motor_map = [0, 0, kT, -kt, -kt,  km; ...
                           0, 0, kT,  kt,  kt,  km; ...
                           0, 0, kT,  kt, -kt, -km; ...
                           0, 0, kT, -kt,  kt, -km];
        
        case 'X6'
            disp('Using PixHawk X6 motor layout')
            
            %   3   6
            %    \ /
            % 2---x---1
            %    / \
            %   5   4
            
            arm_ang = pi/3;
            
            p.frame.map = [  -(arm_ang + arm_ang/2); ...
                                arm_ang + arm_ang/2; ...
                              arm_ang/2; ...
                              -(pi - arm_ang/2); ...
                              -arm_ang/2; ...
                              pi - arm_ang/2];
                          
            nm = numel(p.frame.map);
            
            thrust_scale = p.control.base_link.thrust.reserve;
            la = p.frame.motor_radius;
            p.frame.arm_angle = arm_ang;
            max_thrust = p.motor.max_thrust;
            p.motor.num = nm;
            
            t_x = thrust_scale * max_thrust * ( ...
                      2 * ( cos(pi/2 - arm_ang) * la ) ...
                      + ( la ) ...
                  );
              
            t_y = thrust_scale * max_thrust * ( ...
                      2 * ( cos(arm_ang) * la  ) ...
                  );

            p.control.base_link.torque.x_max = t_x;
            p.control.base_link.torque.y_max = t_y;
            
            p.control.base_link.thrust.max = nm * max_thrust * ...
                                             ( 1.0 - p.control.base_link.thrust.reserve );
            p.control.base_link.thrust.min = 0; % This could be potentially used for negative thrust platforms
                    
            kT = 1/(nm*max_thrust);
            ktx = 1/(2*la*(2*sin(arm_ang/2)+1)*max_thrust);
            kty = 1/(4*la*cos(arm_ang)*max_thrust);
            km = 1/(nm*p.motor.prop_drag);

            p.motor_map = [0, 0, kT, -ktx,    0, -km; ...
                           0, 0, kT,  ktx,    0,  km; ...
                           0, 0, kT,  ktx, -kty, -km; ...
                           0, 0, kT, -ktx,  kty,  km; ...
                           0, 0, kT, -ktx, -kty,  km; ...
                           0, 0, kT,  ktx,  kty, -km];
        
        
        otherwise
            error('Unsupported motor layout')
    end
    
    
end

