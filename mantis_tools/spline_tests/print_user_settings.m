function [ ] = print_user_settings(t0, cdt, dt, tf, frame_type, manip_type, n, order, nvias, smethod, tbase, tjoints, cmethod, cact, w0p, w0t, w0r)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    disp('--== Settings ==--')
    disp(['Timings [t0;cdt/dt;tf]:     [', num2str(t0), ';', num2str(cdt), ';', num2str(dt), ';', num2str(tf), ']'])
    
    disp(['Platform [base;manip;n]:    [', frame_type, ';', manip_type, ';', num2str(n), ']'])
    
    
    actuated = 'under';
    if cact
        actuated = 'full';
    end
    disp(['Control [method;actuation]: [', cmethod, ';', actuated, ']'])
    disp(['Tuning [w0p;w0t;w0r]:       [', num2str(w0p), ';', num2str(w0t), ';', num2str(w0r), ']'])
    
    disp(['Spline [order;vias;method]: [', num2str(order), ';', num2str(nvias), ';', smethod, ']'])

    disp('Trajectories:')
    disp(['    - ', tbase])
    for i = 1:length(tjoints)
        disp(['    - ', tjoints{i}])
    end
    
    disp(' ')
end

