% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ results ] = mantis_sim_simulate( config, x0_overrides, camera )
%MANTIS_SIM_SIMULATE Summary of this function goes here
%   Detailed explanation goes here

    %% Data Generation

    % Create a lookup for all the state variables to make it easier to address
    % all sorts of indexes in our arrays
    sn = state_names_lookup(config.model.n);

    model_d = gen_model(config.model.frame_type, config.model.manip_type, config.model.n, camera);
    model_c = model_d; % Mass properties for controller

    controller_rate = floor((1/config.time.dt)/(1/config.time.cdt));

    [vpx, vpy, vpz, vppsi] = gen_trajectory_vias(config.spline.tname_base, config.spline.num_vias);
    vpr = cell(config.model.n,1);
    r0 = zeros(config.model.n,1);
    for i = 1:config.model.n
        vpr{i} = gen_trajectory_vias_r(config.spline.tname_r{i}, config.spline.num_vias);
        r0(i) = vpr{i}(1); % Easier to capture initial joint states in this loop
    end

    % Initial state
    x0 = [eul2quat([vppsi(1),0,0])';    % Rotation (World)
          vpx(1); vpy(1); vpz(1);       % Position (World)
          r0;                           % Joint Positions (Body)
          zeros(3,1);                   % Linear Velocity (World)
          zeros(3,1);                   % Angular Velocity (Body)
          zeros(3,1);                   % Linear Velocity (Body)
          zeros(config.model.n,1)];     % Joint Velocity (Body)

    for i = 1:size(x0_overrides,1)
        x0(sn.(x0_overrides{i,1})) = x0_overrides{i,2};
    end
    
    % Rest of simulator setup
    disp('Generating...')
    
    t = config.time.t0:config.time.dt:config.time.tf;
    t_vias = linspace(config.time.t0,config.time.tf,config.spline.num_vias);

    if logical(mod(t_vias,config.time.dt))
        error('t_vias entries must be a multiple of dt')
    end

    vias_x = gen_vias(config.spline.dvia_est_method, vpx, t_vias);
    vias_y = gen_vias(config.spline.dvia_est_method, vpy, t_vias);
    vias_z = gen_vias(config.spline.dvia_est_method, vpz, t_vias);
    vias_psi = gen_vias(config.spline.dvia_est_method, vppsi, t_vias);
    vias_r = cell(config.model.n,1);
    for i = 1:config.model.n
        vias_r{i} = gen_vias(config.spline.dvia_est_method, vpr{i}, t_vias);
    end
    [st, s_x] = gen_spline(t_vias, vias_x, config.spline.order, config.time.dt);
    [~, s_y] = gen_spline(t_vias, vias_y, config.spline.order, config.time.dt);
    [~, s_z] = gen_spline(t_vias, vias_z, config.spline.order, config.time.dt);
    [~, s_psi] = gen_spline(t_vias, vias_psi, config.spline.order, config.time.dt);
    s_r = cell(config.model.n,1);
    for i = 1:config.model.n
        [~, s_r{i}] = gen_spline(t_vias, vias_r{i}, config.spline.order, config.time.dt);
    end

    KxP = config.tuning.w0p^2;    % Position tracking P gain
    KxD = 2*config.tuning.w0p;    % Velocity tracking P gain
    KtP = config.tuning.w0t^2;    % Angular tracking P gain
    KtD = 2*config.tuning.w0t;    % Angular rate tracking P gain
    KrP = config.tuning.w0r^2;    % Joint tracking P gain
    KrD = 2*config.tuning.w0r;    % Joint rate tracking P gain
    
    KxI = 0;
    if config.control.use_pos_int > 0
        KxI = 0.5*KxP;
    end

    % Sanity check:
    % Ensure that our time vectors ended up the same size, and same dt,
    % probably fine if these two cases line up
    timescale_checks = logical([(size(st) ~= size(t)), ...
                                ((st(2)-st(1)) ~= (t(2)-t(1)))]);
    if timescale_checks
        error('Error constructing spline times')
    end

    % Build state vector
    x = zeros(size(x0,1),length(t));
    x(:,1) = x0;

    % Build control vector
    c = zeros(20+3*config.model.n-1,length(t));
    c(sn.CONTROL_P_B,1) = x0(sn.STATE_XYZ);


    %% Simulation
    reverseStr = '';
    progressLast = 0;

    control_tick = 0;
    control_tau_last = zeros(length(sn.STATE_REDUCED),1);

    % Controller integrator variables
    pos_integrator = zeros(3,1);
    npid_ang_integrator = zeros(3,1);
    npid_omega_integrator = zeros(3,1);
    manip_integrator = zeros(config.model.n,1);
    
    % Aerodynamic properties (if wind enabled)
    aero.Cd = 0.8;
    aero.rho = 1.225;
    aero.mass = calc_total_mass(model_d);
    aero.A = (0.5*0.05) + config.model.n*(0.25*0.01);
    aero.do_aero = sum(abs(config.wind)) > 0;
    
    % Fill in the control inputs for the first time step (to avoid analysis
    % issues)
    c(sn.CONTROL_A,1) =  -config.g_vec;
    c(sn.CONTROL_Q,1) = x0(sn.STATE_Q);
    c(sn.CONTROL_WXYZ_B,1) = x0(sn.STATE_WXYZ_B);
    c(sn.CONTROL_DWXYZ_B,1) = zeros(size(sn.CONTROL_DWXYZ_B));
    c(sn.CONTROL_R,1) = x0(sn.STATE_R);
    c(sn.CONTROL_RD,1) = x0(sn.STATE_RD);
    c(sn.CONTROL_RDD,1) = zeros(size(sn.CONTROL_RDD));
    c(sn.CONTROL_TAU,1) = zeros(size(sn.CONTROL_TAU));
    
            
    for i = 2:length(t)
        %% Status Messages
        progress = 100*i/length(t);
        if progress > progressLast + 0.1
            msg = sprintf('Simulating... %3.1f%', progress);
            fprintf([reverseStr, msg]);
            reverseStr = repmat(sprintf('\b'), 1, length(msg));
            progressLast = progress;
        end
        
        % Previous rotation matrix (for ease of use)
        R_p = quat2rotm(x(sn.STATE_Q,i-1)');

        pos_e = zeros(3,1);
        
        if control_tick <= 1
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

            if config.control.tracking_frame == 0
                % Base Tracking
                % Projection to SO(3)/so(3)
                [R_sp_s, w_sp_s, wd_sp_s] = map_angles_rates_accels(acc, jerk, snap, yaw, dyaw, ddyaw);
                pos_b = pos;
                vel_b = vel;
                acc_b = acc;
            elseif config.control.tracking_frame > config.model.n
                % End effector tracking
                pos_e = pos;
                vel_e = vel;
                
                Psi_b = [cos(yaw), -sin(yaw), 0;
                         sin(yaw),  cos(yaw), 0;
                                0,         0, 1];

                [X_be, eta_be, etad_be] = FKfly(model_c, config.model.n+1, r, rd, rdd);
                g_be = inverse_trans(pluho(X_be));
                R_be = g_be(1:3,1:3);
                
                R_e = Psi_b*R_be;
                gt_e = [R_e, pos_e;
                        zeros(1,3),   1];
                g_b = gt_e*inverse_trans(g_be);

                pd_be = R_e*eta_be(4:6);
                pdd_be = R_e*etad_be(4:6);

                pos_b = g_b(1:3,4);
                vel_b = vel_e - pd_be;
                acc_b = acc - pdd_be;
                
                R_sp_s = Psi_b;
                w_sp_s = zeros(3,1);
                wd_sp_s = zeros(3,1);
            else
                error('Tracking intermediate frames has not been implemented');
            end

            %% Control Law            
            if strcmp(config.control.method, 'npid_px4')
                if i > 2
                    x_p = x(:,i-2);
                else
                    x_p = x(:,i-1);
                end

                [tau_b, acc_c, q_sp_c, pos_integrator, npid_omega_integrator ] = control_nonlinear_pid_px4( model_c, ...          
                                                         pos_b, vel_b, acc_b, yaw, ...
                                                         x(:,i-1), x_p, ...
                                                         config.time.cdt, pos_integrator, npid_omega_integrator, ...
                                                         KxP, KxI, KxD);

                [tau_r, manip_integrator] = control_manip_decoupled( model_c, ...
                                            r, rd, ...
                                            x(:,i-1), ...
                                            config.time.cdt, manip_integrator);

                tau_full = [tau_b;tau_r];

            elseif strcmp(config.control.method, 'npid_exp')
    %             if i > 2
    %                 x_p = x(:,i-2);
    %             else
    %                 x_p = x(:,i-1);
    %             end
    %             
    %             [tau_full, acc_c, q_sp_c, npid_ang_integrator, npid_omega_integrator ] = control_nonlinear_pid_exp( model_c, ...          
    %                                                      pos_b, vel_b, acc_b, yaw, ...
    %                                                      x(:,i-1), x_p, ...
    %                                                      cdt, npid_ang_integrator, npid_omega_integrator);
            elseif strcmp(config.control.method, 'npid')
    %             % Instead of normalising the thrust vector, simply use F=ma
    %             kT = mass_c.m;
    %             [tau_full, acc_c, q_sp_c, npid_ang_integrator ] = control_nonlinear_pid( model_c, ...          
    %                                                      pos_b, vel_b, acc_b, ...
    %                                                      yaw, w_sp_s, ...
    %                                                      x(:,i-1), kT, ...
    %                                                      KxP, KxD, KtP, KtD, yaw_w, ...
    %                                                      cdt, npid_ang_integrator);
            elseif strcmp(config.control.method, 'ctc')
                [tau_full, acc_c, q_sp_c, pos_integrator ] = control_computed_torque( model_c, config.time.cdt, ...
                                             pos_b, vel_b, acc_b, ...
                                             yaw, R_sp_s, w_sp_s, wd_sp_s, ...
                                             r, rd, rdd, ...
                                             x(:,i-1), ...
                                             KxP, KxD, ...
                                             KxI, pos_integrator, ...
                                             KtP, KtD, config.control.yaw_w, ...
                                             KrP, KrD );
            elseif strcmp(config.control.method, 'feed')
                if i > 2
                    x_p = x(:,i-2);
                else
                    x_p = x(:,i-1);
                end

                [tau_b, acc_c, q_sp_c, pos_integrator, npid_omega_integrator ] = control_feed_forward( model_c, ...
                                             pos_b, vel_b, acc_b, yaw, ...
                                             x(:,i-1), x_p, ...
                                             config.time.cdt, pos_integrator, npid_omega_integrator, ...
                                             KxP, KxI, KxD);

                [tau_r, manip_integrator] = control_manip_decoupled( model_c, ...
                                            r, rd, ...
                                            x(:,i-1), ...
                                            config.time.cdt, manip_integrator);
                tau_full = [tau_b;tau_r];
            else
                error('Could not determine control method to use')
            end

            % Handle fully/under actuated cases
            if config.control.fully_actuated > 0
                tau = tau_full;
            else
                tau = [tau_full(1:3);0;0;tau_full(6:end)];
            end


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


            %% Control loop management
            % Save control signal for next step
            control_tau_last = tau;
            % Set the control loop run again shortly
            control_tick = controller_rate;


        else
            % Use the control signal from the last control tick
            tau = control_tau_last;
            c(sn.CONTROL_P_B,i) = c(sn.CONTROL_P_B,i-1);
            c(sn.CONTROL_P_E,i) = c(sn.CONTROL_P_E,i-1);
            c(sn.CONTROL_A,i) = c(sn.CONTROL_A,i-1);
            c(sn.CONTROL_Q,i) = c(sn.CONTROL_Q,i-1);
            c(sn.CONTROL_WXYZ_B,i) = c(sn.CONTROL_WXYZ_B,i-1);
            c(sn.CONTROL_DWXYZ_B,i) = c(sn.CONTROL_DWXYZ_B,i-1);
            c(sn.CONTROL_R,i) = c(sn.CONTROL_R,i-1);
            c(sn.CONTROL_RD,i) = c(sn.CONTROL_RD,i-1);
            c(sn.CONTROL_RDD,i) = c(sn.CONTROL_RDD,i-1);
            c(sn.CONTROL_TAU,i) = c(sn.CONTROL_TAU,i-1);

            control_tick = control_tick - 1;
        end


        %% Simulte Time Step
        dx = mantis_sim_run(x(:,i-1), tau, model_d);
        
        
        % Calculate aerodynamic effects if wind present
        aero_acc = zeros(3,1);
        if aero.do_aero > 0
            wind_v = config.wind; %- x(sn.STATE_VXYZ,i-1);
            % Use [v.*abs(v)] instead of [v.^2] to preserve direction
            aero_drag = 0.5*aero.rho*aero.A*aero.Cd*(wind_v.*abs(wind_v));
            aero_acc = aero_drag*aero.mass;
        end


        % Update Dynamics

        % Joints
        x(sn.STATE_RD,i) = x(sn.STATE_RD,i-1) + dx(sn.STATE_REDUCED_RD)*config.time.dt;
        x(sn.STATE_R,i) = x(sn.STATE_R,i-1) + x(sn.STATE_RD,i-1)*config.time.dt;


        % Angular Velocity
        x(sn.STATE_WXYZ_B,i) = x(sn.STATE_WXYZ_B,i-1) + dx(sn.STATE_REDUCED_W_B)*config.time.dt;
    %     % Perfect angular velocity tracking
    %     x(sn.STATE_WXYZ_B,i) = R_p'*R_sp_s*c(sn.CONTROL_WXYZ_B,i); %XXX: Need to pull into the current frame of reference

        % Rotation
        R = orthagonalize_rotm(R_p + R_p*vee_up(x(sn.STATE_WXYZ_B,i-1)*config.time.dt));
        x(sn.STATE_Q,i) = rotm2quat(R)';

        % Linear Velocity
        
        % Propogate linear velocity in the world frame
        x(sn.STATE_VXYZ,i) = x(sn.STATE_VXYZ,i-1) + (config.g_vec + aero_acc + R_p*dx(sn.STATE_REDUCED_V_B))*config.time.dt;
        % Then convert back to the body frame for the current time step
        x(sn.STATE_VXYZ_B,i) = R'*x(sn.STATE_VXYZ,i);

        % Linear Position
        x(sn.STATE_XYZ,i) = x(sn.STATE_XYZ,i-1) + x(sn.STATE_VXYZ,i-1)*config.time.dt;
    end

    disp(' ') % Add some spacing, needed due to previous printf()
    disp(' ')
    
    
    %% Prepare Results
    
    results.model_c = model_c;
    results.model_d = model_d;
    results.t = t;
    results.x = x;
    results.c = c;
    
    results.traj.via_steps = ((config.time.tf - config.time.t0) / config.time.dt) / (config.spline.num_vias - 1);
    results.traj.num_traj_points = config.spline.num_vias - 1;
    results.traj.t_vias = t_vias;
    results.traj.vpx = vpx;
    results.traj.vpy = vpy;
    results.traj.vpz = vpz;
    results.traj.vppsi = vppsi;
    results.traj.vias_x = vias_x;
    results.traj.vias_y = vias_y;
    results.traj.vias_z = vias_z;
    results.traj.vias_psi = vias_psi;
    results.traj.vias_r = vias_r;
    results.traj.s_x = s_x;
    results.traj.s_y = s_y;
    results.traj.s_z = s_z;
    results.traj.s_psi = s_psi;
    results.traj.s_r = s_r;
    
    results.gains.KxP = KxP;
    results.gains.KxD = KxD;
    results.gains.KtP = KtP;
    results.gains.KtD = KtD;
    results.gains.KrP = KrP;
    results.gains.KrD = KrD;
    
end

