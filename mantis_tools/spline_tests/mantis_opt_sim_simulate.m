function [ results ] = mantis_opt_sim_simulate( config, camera, vias, num_samples )
%MANTIS_SIM_SIMULATE Summary of this function goes here
%   Detailed explanation goes here

    %% Data Generation

    % Create a lookup for all the state variables to make it easier to address
    % all sorts of indexes in our arrays
    sn = state_names_lookup(config.model.n);

    model_d = gen_model(config.model.frame_type, config.model.manip_type, config.model.n, camera);
    model_c = model_d; % Mass properties for controller


    % Rest of simulator setup
%     disp('Generating...')
    
%     t = config.time.t0:config.time.dt:config.time.tf;
    %t_vias = linspace(config.time.t0,config.time.tf,config.spline.num_vias);
%     t_vias = t(floor(linspace(1,length(t),config.spline.num_vias)));
% 
%     if logical(mod(vias.time,config.time.dt))
%         error('t_vias entries must be a multiple of dt')
%     end

%     [t, s_x] = gen_spline_samples(vias.time, vias.x, config.spline.order, num_samples);
%     [~, s_y] = gen_spline_samples(vias.time, vias.y, config.spline.order, num_samples);
%     [~, s_z] = gen_spline_samples(vias.time, vias.z, config.spline.order, num_samples);
%     [~, s_psi] = gen_spline_samples(vias.time, vias.psi, config.spline.order, num_samples);
%     s_r = cell(config.model.n,1);
%     for i = 1:config.model.n
%         [~, s_r{i}] = gen_spline_samples(vias.time, vias.r{i}, config.spline.order, num_samples);
%     end

    [ t, s_x, s_y, s_z, s_psi, s_r ] = mantis_opt_sim_spline_gen( config, vias, num_samples );
    
    % Sanity check:
    % Ensure that our time vectors ended up the same size, and same dt,
    % probably fine if these two cases line up
%     timescale_checks = logical([(size(st) ~= size(t)), ...
%                                 ((st(2)-st(1)) ~= (t(2)-t(1)))]);
%     if timescale_checks
%         error('Error constructing spline times')
%     end

    % Build control vector
    c = zeros(20+3*config.model.n-1,length(t));
%     c(sn.CONTROL_P_B,1) = x0(sn.STATE_XYZ);


    %% Simulation
    reverseStr = '';
    progressLast = 0;
    
    % Fill in the control inputs for the first time step (to avoid analysis
    % issues)
%     c(sn.CONTROL_A,1) =  -config.g_vec;
%     c(sn.CONTROL_Q,1) = x0(sn.STATE_Q);
%     c(sn.CONTROL_WXYZ_B,1) = x0(sn.STATE_WXYZ_B);
%     c(sn.CONTROL_DWXYZ_B,1) = zeros(size(sn.CONTROL_DWXYZ_B));
%     c(sn.CONTROL_R,1) = x0(sn.STATE_R);
%     c(sn.CONTROL_RD,1) = x0(sn.STATE_RD);
%     c(sn.CONTROL_RDD,1) = zeros(size(sn.CONTROL_RDD));
%     c(sn.CONTROL_TAU,1) = zeros(size(sn.CONTROL_TAU));
%     
            
    for i = 1:length(t)
%         %% Status Messages
%         progress = 100*i/length(t);
%         if progress > progressLast + 0.1
%             msg = sprintf('Simulating... %3.1f%', progress);
%             fprintf([reverseStr, msg]);
%             reverseStr = repmat(sprintf('\b'), 1, length(msg));
%             progressLast = progress;
%         end
%         
%         % Previous rotation matrix (for ease of use)
%         R_p = quat2rotm(x(sn.STATE_Q,i-1)');

        pos_e = zeros(3,1);
        
        %% Guidance
        % Spline reference vectors
        pos = [s_x(sn.SPLINE_POS,i);s_y(sn.SPLINE_POS,i);s_z(sn.SPLINE_POS,i)];
        vel = [s_x(sn.SPLINE_VEL,i);s_y(sn.SPLINE_VEL,i);s_z(sn.SPLINE_VEL,i)];
        acc = [s_x(sn.SPLINE_ACC,i);s_y(sn.SPLINE_ACC,i);s_z(sn.SPLINE_ACC,i)] - config.g_vec; %XXX: -g to have a positive term for gravity added
        jerk = [s_x(sn.SPLINE_JERK,i);s_y(sn.SPLINE_JERK,i);s_z(sn.SPLINE_JERK,i)];
        snap = [s_x(sn.SPLINE_SNAP,i);s_y(sn.SPLINE_SNAP,i);s_z(sn.SPLINE_SNAP,i)];

        yaw = s_psi(sn.SPLINE_POS,i);
        dyaw = s_psi(sn.SPLINE_VEL,i);
        ddyaw = s_psi(sn.SPLINE_ACC,i);

        r = zeros(config.model.n,1);
        rd = zeros(config.model.n,1);
        rdd = zeros(config.model.n,1);
        for j = 1:config.model.n
            r(j) = s_r{j}(sn.SPLINE_POS,i);
            rd(j) = s_r{j}(sn.SPLINE_VEL,i);
            rdd(j) = s_r{j}(sn.SPLINE_ACC,i);
        end

        % Base Tracking
        % Projection to SO(3)/so(3)
        [R_sp_s, w_sp_s, wd_sp_s] = map_angles_rates_accels(acc, jerk, snap, yaw, dyaw, ddyaw);
        pos_b = pos;
        vel_b = vel;
        acc_b = acc;
        
        acc_c = acc_b;
        q_sp_c = rotm2quat(R_sp_s);

        % Feedback Linearisation
        xid_b = [wd_sp_s;
                 acc_b];

        [tauf, taur] = IDfly( model_d, zeros(6,1), xid_b, r, zeros(config.model.n,1), rdd ); 

        tau = [tauf; ...
               taur];
        
        %% Save control inputs
        c(sn.CONTROL_P_B,i) = pos_b;
        c(sn.CONTROL_P_E,i) = pos_e;
        c(sn.CONTROL_A,i) = acc_c;
        c(sn.CONTROL_Q,i) = q_sp_c';
        c(sn.CONTROL_WXYZ_B,i) = w_sp_s;
        c(sn.CONTROL_DWXYZ_B,i) = wd_sp_s;
        c(sn.CONTROL_R,i) = r;
        c(sn.CONTROL_RD,i) = rd;
        c(sn.CONTROL_RDD,i) = rdd;
        c(sn.CONTROL_TAU,i) = tau;
    end

    
    %% Prepare Results
    
    results.model_c = model_c;
    results.model_d = model_d;
    results.t = t;
%     results.x = x;
    results.c = c;
    
    results.traj.via_steps = ((config.time.tf - config.time.t0) / config.time.dt) / (config.spline.num_vias - 1);
    results.traj.num_traj_points = config.spline.num_vias - 1;
    results.traj.s_x = s_x;
    results.traj.s_y = s_y;
    results.traj.s_z = s_z;
    results.traj.s_psi = s_psi;
    results.traj.s_r = s_r;

    
end

