function [ analysis ] = mantis_sim_analyse( config, results, print_latex_results )
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

    pos_ref = [results.traj.s_x(1,:);results.traj.s_y(1,:);results.traj.s_z(1,:)];
    pos_error = pos_ref - results.x(sn.STATE_XYZ,:);
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
    
%     u0 = config.g*calc_total_mass(results.model_d);
    analysis.performance.J_fuel = sum(sum(abs(results.c(sn.CONTROL_TAU,:))))*config.time.dt;
    
    for i = length(results.t)-1:-1:1
        ts_r = 0.05; % 5% settling time range
        % Create scaling values (or use a preset if sp is zero)
        p_sp = pos_ref(:,i);
        p_sp(~p_sp) = 1.0;
        tv_sp = tv_S_ref(i);
        tv_sp(~tv_sp) = pi;
        r_sp = results.c(sn.CONTROL_R,i);
        r_sp(~r_sp) = pi/2;
        
        ne_pos = pos_error(:,i) ./ p_sp;
        ne_ang = analysis.tv_error(i) ./ tv_sp;
        ne_r = r_error(i) ./ r_sp;
        
        check = abs([ne_pos; ne_ang; ne_r]) > ts_r;
        
        if check
            analysis.performance.ts = results.t(i+1);
            break;
        end
    end
    
    if print_latex_results > 0
        ldf = '%0.3f';
        disp('Latex results aligned as [Max Pos. Error, Pos. RMSE, Omega RMSE] as [x,y,z]:')
        disp(['    ', num2str(analysis.max_pos_error(1), ldf), ' & ', num2str(analysis.max_pos_error(2), ldf), ' & ', num2str(analysis.max_pos_error(3), ldf), ' & ' ...
              num2str(analysis.pos_RMSE(1), ldf), ' & ', num2str(analysis.pos_RMSE(2), ldf), ' & ', num2str(analysis.pos_RMSE(3), ldf), ' & ' ...
              num2str(analysis.w_RMSE(1), ldf), ' & ', num2str(analysis.w_RMSE(2), ldf), ' & ', num2str(analysis.w_RMSE(3), ldf), ' \\']);
        disp(' ')
    else
        disp('Max Position Errors:')
        disp(analysis.max_pos_error);
        disp('Position RMSE:')
        disp(analysis.pos_RMSE)
        disp('Omega RMSE:')
        disp(analysis.w_RMSE)
        disp('Joint RMSE:')
        disp(analysis.r_RMSE)
        disp('Total Control Cost:')
        disp(analysis.performance.J_fuel)
        disp('5% Settling Time:')
        disp(analysis.performance.ts)
    end
end

