function [ ] = print_config(config)
%PRINT_CONFIG Summary of this function goes here
%   Detailed explanation goes here

    disp('--== Settings ==--')
    disp(['Timings [t0;cdt/dt;tf]:     [', num2str(config.time.t0), ';', ...
                                           num2str(config.time.cdt), ';', ...
                                           num2str(config.time.dt), ';', ...
                                           num2str(config.time.tf), ']'])
    
    disp(['Platform [base;manip;n]:    [', config.model.frame_type, ';', ...
                                           config.model.manip_type, ';', ...
                                           num2str(config.model.n), ']'])
    
    
    actuated = 'under';
    if config.control.fully_actuated
        actuated = 'full';
    end
    disp(['Control [method;actuation]: [', config.control.method, ';', ...
                                           actuated, ']'])
    disp(['        [yaw_w,theta_max]:  [', num2str(config.control.yaw_w), ';', ...
                                                    num2str(config.control.theta_max), ']'])
    
    disp(['Tuning [w0p;w0t;w0r]:       [', num2str(config.tuning.w0p), ';', ...
                                           num2str(config.tuning.w0t), ';', ...
                                           num2str(config.tuning.w0r), ']'])
    
    disp(['Spline [order;vias;method]: [', num2str(config.spline.order), ';', ...
                                           num2str(config.spline.num_vias), ';', ...
                                           config.spline.dvia_est_method, ']'])

    disp('Trajectories:')
    disp(['    - ', config.spline.tname_base])
    for i = 1:length(config.spline.tname_r)
        disp(['    - ', config.spline.tname_r{i}])
    end
    
    disp(' ')
end

