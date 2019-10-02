function [ analysis ] = mantis_sim_analyse( config, results )
%MANTIS_SIM_SIMULATE Summary of this function goes here
%   Detailed explanation goes here

    sn = state_names_lookup(config.model.n);

    eul = quat2eul(results.x(sn.STATE_Q,:)', 'ZYX');
    analysis.roll = eul(:,3);
    analysis.pitch = eul(:,2);
    analysis.yaw = eul(:,1);

    % eul_c = rotm2eul(reshape(c(sn.CONTROL_R,:),3,3,length(t)), 'ZYX');
    eul_c = quat2eul(results.c(sn.CONTROL_Q,:)', 'ZYX');
    analysis.roll_c = eul_c(:,3);
    analysis.pitch_c = eul_c(:,2);
    analysis.yaw_c = eul_c(:,1);
    
    analysis.traj.ref_traj_dir = zeros(3,length(results.traj.vias_psi(sn.SPLINE_POS,:)));
    yawR = eul2rotm([results.traj.vias_psi(sn.SPLINE_POS,:);
                     zeros(size(results.traj.vias_psi(sn.SPLINE_POS,:)));
                     zeros(size(results.traj.vias_psi(sn.SPLINE_POS,:)))]');
    for i = 1:length(results.traj.vias_psi(sn.SPLINE_POS,:))
        analysis.traj.ref_traj_dir(:,i) = yawR(:,:,i)*[1;0;0];
    end
    
%     pos_ref = [results.traj.s_x(1,:);results.traj.s_y(1,:);results.traj.s_z(1,:)];
    pos_error = results.c(sn.CONTROL_P_B,:) - results.x(sn.STATE_XYZ,:);
    analysis.max_pos_error = max(abs(pos_error),[],2);
    analysis.pos_RMSE = sqrt(mean((pos_error).^2,2));

    w_error = results.c(sn.CONTROL_WXYZ_B,:) - results.x(sn.STATE_WXYZ_B,:);
    analysis.w_RMSE = sqrt(mean((w_error).^2,2));
    
    r_error = results.c(sn.CONTROL_R,:) - results.x(sn.STATE_R,:);
    analysis.r_RMSE = sqrt(mean((r_error).^2,2));

    R_state = quat2rotm(results.x(sn.STATE_Q,:)');
    tv_R_state = squeeze(R_state(:,3,:));
    tv_spline = [results.traj.s_x(sn.SPLINE_ACC,:);results.traj.s_y(sn.SPLINE_ACC,:);results.traj.s_z(sn.SPLINE_ACC,:)] - config.g_vec;
    tv_S_ref = tv_spline./vecnorm(tv_spline,2,1);
    analysis.tv_error = thrustvec_angle_error( tv_S_ref, tv_R_state);
    
    analysis.end_effector_pos.ref = zeros(3,length(results.t));
    analysis.end_effector_pos.state = zeros(3,length(results.t));
    for i = 1:length(results.t)
        g_be = inverse_trans(pluho(FKfly(results.model_d, config.model.n+1, results.x(sn.STATE_R,i), [], [])));
%         g_b = [quat2rotm(x(sn.STATE_Q)'), x(sn.STATE_XYZ);
%                zeros(1,3), 1];
        g_b = [R_state(:,:,i), results.x(sn.STATE_XYZ,i);
               zeros(1,3), 1];
        g_e = g_b*g_be;
        analysis.end_effector_pos.state(:,i) = g_e(1:3,4);
        
        if i == 1
            % no ref on first pass
            analysis.end_effector_pos.ref(:,i) = analysis.end_effector_pos.state(:,i);
        else
            analysis.end_effector_pos.ref(:,i) = results.c(sn.CONTROL_P_E,i);
        end
    end
    
%     u0 = config.g*calc_total_mass(results.model_d);
    analysis.performance.J_fuel = sum(sum(abs(results.c(sn.CONTROL_TAU,:))))*config.time.dt;
    
    analysis.performance.ts = NaN;
    for i = length(results.t)-1:-1:1
        ts_r = 0.05; % 5% settling time range
        % Create scaling values (or use a preset if sp is zero)
        
%         p_sp = pos_ref(:,i);
%         p_sp(~p_sp) = 1.0;
%         r_sp = results.c(sn.CONTROL_R,i);
%         r_sp(~r_sp) = pi/2;
        p_sp = 1.0;
        r_sp = pi/2;
        
        ne_pos = pos_error(:,i) ./ p_sp;
        ne_ang = analysis.tv_error(i) ./ pi;
        ne_r = r_error(:,i) ./ r_sp;
        
        check = abs([ne_pos; ne_ang; ne_r]) > ts_r;
        
        if any(check)
            % If we fail the check, then we have reached the index where we
            % are no longer within 5% (working backwards). As long as this
            % isn't the first check (i.e. it never reached 5% settling),
            % then we save the settling time.
            if (i+1) < length(results.t)
                analysis.performance.ts = results.t(i+1);
            end
            
            break;
        end
    end
    % If we reached the end of the loop, then we never diverged from
    % settled state
    if i == 1
        analysis.performance.ts = config.time.t0;
    end
    
    tdf = '%0.2f ';
    ldf = '%0.5f ';
    disp(['    5% Settling Time:    ', num2str(analysis.performance.ts,tdf)])
    disp(['    Total Control Cost:  ', num2str(analysis.performance.J_fuel,tdf)])
    disp(['    Max Position Errors: ', num2str(analysis.max_pos_error',ldf)])
    disp(['    Position RMSE:       ', num2str(analysis.pos_RMSE',ldf)])
    disp(['    Omega RMSE:          ', num2str(analysis.w_RMSE',ldf)])
    disp(['    Joint RMSE:          ', num2str(analysis.r_RMSE',ldf)])
end

