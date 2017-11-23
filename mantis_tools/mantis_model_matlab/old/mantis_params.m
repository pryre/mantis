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
        case 'X6'
            disp('Using PixHawk X6 motor layout')
            
            thrust_scale = p.control.base_link.thrust.reserve;
            arm_len = p.frame.motor_arm_length;
            max_thrust = p.motor.max_thrust;
            r30 = deg2rad(30);
            r60 = deg2rad(60);
            
            t_x = thrust_scale * ( ...
                      2 * ( cos(r30) * arm_len *  thrust_scale * max_thrust ) ...
                      + ( arm_len *  thrust_scale * max_thrust ) ...
                  );
              
            t_y = thrust_scale * ( ...
                      2 * ( cos(r60) * arm_len *  thrust_scale * max_thrust ) ...
                  );

            p.control.base_link.torque.x_max = t_x;
            p.control.base_link.torque.y_max = t_y;
            
            p.control.base_link.thrust.max = 6 * max_thrust * ...
                                             ( 1.0 - p.control.base_link.thrust.reserve );
            p.control.base_link.thrust.min = 0; % This could be potentially used for negative thrust platforms
        otherwise
            error('Unsupported motor layout')
    end
    
    
end

