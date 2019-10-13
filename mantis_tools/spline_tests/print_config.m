function [ ] = print_config(config)
%PRINT_CONFIG Summary of this function goes here
%   Detailed explanation goes here

    disp('--== Settings ==--')
    disp(['Gavity:                     [', num2str(config.g_vec(1)),';',num2str(config.g_vec(2)),';',num2str(config.g_vec(3)),']']);
    disp(['Wind:                       [', num2str(config.wind(1)),';',num2str(config.wind(2)),';',num2str(config.wind(3)),']']);
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
    pos_int = 'no';
    if config.control.use_pos_int
        pos_int = 'yes';
    end
    disp(['Control [method;pint;fact]: [', config.control.method, ';', ...
                                           pos_int, ';', ...
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

