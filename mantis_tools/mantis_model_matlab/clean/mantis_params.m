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
            
            p.frame.arm_angle = pi/4;
            
            thrust_scale = p.control.base_link.thrust.reserve;
            arm_len = p.frame.motor_radius;
            arm_ang = p.frame.arm_angle;
            max_thrust = p.motor.max_thrust;
            
            p.frame.map = [  -arm_ang; ...
                            3*arm_ang; ...
                              arm_ang; ...
                           -3*arm_ang];
                       
            p.motor.num = numel(p.frame.map);
            
            t_x = thrust_scale * max_thrust * ( ...
                      2 * ( cos(arm_ang) * arm_len  ) ...
                  );
              
            t_y = thrust_scale * max_thrust * ( ...
                      2 * ( sin(arm_ang) * arm_len ) ...
                  );

            p.control.base_link.torque.x_max = t_x;
            p.control.base_link.torque.y_max = t_y;
            
            p.control.base_link.thrust.max = p.motor.num * max_thrust * ...
                                             ( 1.0 - p.control.base_link.thrust.reserve );
            p.control.base_link.thrust.min = 0; % This could be potentially used for negative thrust platforms
        case 'X6'
            disp('Using PixHawk X6 motor layout')
            
            p.frame.arm_angle = pi/3;
            
            thrust_scale = p.control.base_link.thrust.reserve;
            arm_len = p.frame.motor_radius;
            arm_ang = p.frame.arm_angle;
            max_thrust = p.motor.max_thrust;
            
            p.frame.map = [  -(arm_ang + arm_ang/2); ...
                                arm_ang + arm_ang/2; ...
                              arm_ang/2; ...
                              -(pi - arm_ang/2); ...
                              -arm_ang/2; ...
                              pi - arm_ang/2];
                          
            p.motor.num = numel(p.frame.map);
            
            t_x = thrust_scale * max_thrust * ( ...
                      2 * ( cos(pi/2 - arm_ang) * arm_len ) ...
                      + ( arm_len ) ...
                  );
              
            t_y = thrust_scale * max_thrust * ( ...
                      2 * ( cos(arm_ang) * arm_len  ) ...
                  );

            p.control.base_link.torque.x_max = t_x;
            p.control.base_link.torque.y_max = t_y;
            
            p.control.base_link.thrust.max = p.motor.num * max_thrust * ...
                                             ( 1.0 - p.control.base_link.thrust.reserve );
            p.control.base_link.thrust.min = 0; % This could be potentially used for negative thrust platforms
        otherwise
            error('Unsupported motor layout')
    end
    
    
end

