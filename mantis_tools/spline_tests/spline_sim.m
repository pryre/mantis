%% Setup

close all;
clear;
clc;


%% User Variables
% Spline Order:
%   cubic:  3
%   quntic: 5
%   septic: 7
%   nonic:  9
order = 9;

plot_title = "Nontic Spline Reference";

via_steps = 500;
% via_points = sin(linspace(0,2*pi,9));
via_points  = [0,0,1,-2,1,0,0];

t0 = 0;
tf = 10;
%t = linspace(t0, tf, (length(via_points)-1)*via_steps);

mass.m = 1.83497;
mass.Ixx = 0.02961;
mass.Iyy = 0.02933;
mass.Izz = 0.05342;


% Max Error: 
%     0.0573
% 
% Position RMSE: 
%     0.0267
% 
% Omega RMSE: 
%     0.0067


%% Script Variables

x0 = [0;    % Position
      0;    % Velocity
      0;    % Pitch
      0];   % Pitch Rate

KxP = 0.0;%1.0;  % Position tracking P gain
KxdP = 0.0;%2.0;%4.0; % Velocity tracking P gain
% KxdD = 0.5; % Velocity tracking D gain
KtP = 0.0;%3.0;%6.5; % Pitch tracking P gain
KtdP = 0.0;%10.0;%25.0; % Pitch tracking P gain
theta_max = deg2rad(30); % Pitch tracking P gain

g = 9.80665;

sd = (tf - t0) / (length(via_points) - 1);
vias = gen_vias('fdcc', via_points, sd);
t_vias = linspace(t0,tf,length(via_points));

[t, s] = gen_spline(0, tf, vias, order, via_steps);
dt = t(2) - t(1);

% error('Break')

% Build control vector
c = zeros(4,length(t)); %[acc;pitch;pitch_rate;pitch_acc(u)]

% Build state vector
x = zeros(size(x0,1),length(t));
x(:,1) = x0;


%% Simulation
reverseStr = '';
progressLast = 0;

for i = 2:length(t)
    % Display the progress
    progress = 100*i/length(t);
    if progress > progressLast + 0.1
        msg = sprintf('Simulating... %3.1f', progress);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
        progressLast = progress;
    end
    
    % Build acceleration control reference from spline tracking error
    c(1,i) = s(3,i) + KxP*(s(1,i) - x(1,i-1)) + KxdP*(s(2,i) - x(2,i-1));
    
%     % Pitch control reference from acceleration vector
%     c(2,i) = atan2(c(1,i), g);
% 
%     if c(2,i) < -theta_max
%         c(2,i) = -theta_max;
%     end
%     if c(2,i) > theta_max
%         c(2,i) = theta_max;
%     end
% 
%     % Pitch rate control reference
%     c(3,i) = KtP*(c(2,i) - x(3,i-1));
% 
%     % Thrust/motor reference
%     c(4,i) = KtdP*(c(3,i) - x(4,i-1));
%     v = c(4,i);
    
    [~, q_l] = rot_from_vec_yaw([s(3,i-1),0,g],0);
    [~, q_n] = rot_from_vec_yaw([s(3,i),0,g],0);
    e_l = quat2eul(q_l,'ZYX');
    e_n = quat2eul(q_n,'ZYX');
%     qd_l = quat2eul(rot_from_vec_yaw([s(4,i-1),0,g],0),'ZYX') %TODO: Need to get quaternion from rot_from_vec_yaw first
%     q_t = quatnormalize(q_l + qd_l);
    
%     quat2rotm(q_n)

    % Rotation matrix kinematics
    % dR/dt = w_hat*R
    % w_hat = dR/(dt*R)
    % w_hat = dR*R'/dt
    w_hat = (quat2rotm(q_n) - quat2rotm(q_l))/(dt*quat2rotm(q_n));
    w = [w_hat(3,2);
         w_hat(1,3);
         w_hat(2,1)];
     
    % dR/dt = Rd = w_hat*R
%     Rd = rd_from_dvec_dyaw([s(3,i),0,g],[s(4,i),0,0],0,0);
    v1 = [s(3,i),0,g] / norm([s(3,i),0,g]);
    v2_f = [s(3,i),0,g] + [s(4,i),0,0];
    v2 = v2_f / norm(v2_f);
%     theta_d_angle = acos(dot(v1,v2)); %XXX: Not needed due to unit vectors: arcos((v1.v2)/(||v1||*||v2||))
%     theta_d_axis = cross(v1,v2);    %XXX: Not needed due to unit vectors: normalised(v1 x v2)
%     theta_d = theta_d_angle*theta_d_axis/norm(theta_d_axis);
    
    
    acc = [s(3,i),0,g];
    acc_n = acc / norm(acc);
    jerk = [s(4,i),0,0];
    snap = [s(5,i),0,0];
    
    theta_d = cross(acc,jerk) / (norm(acc)^2);
    theta_dd = cross(acc,snap) / (norm(acc)^2);
    
%     j_perp = dot(jerk,acc_n)*acc_n;
%     j_tngt = jerk - v_perp;
    j_tngt = jerk - dot(jerk,acc_n)*acc_n;
    s_tngt = snap - dot(snap,acc_n)*acc_n;
    
    theta_d2 = cross(acc,j_tngt) / (norm(acc)^2);
    theta_dd2 = cross(acc,s_tngt) / (norm(acc)^2);
    
    [theta_d3, theta_dd3] = map_body_rates_accels(acc, jerk, snap);
    
%     disp(['Calc ang: ', num2str(atan2(s(3,i), g)), '; Vec ang: ', num2str(e_n(2))])
%     disp(['Calc w: ', num2str(c(3,i)), '; Vec w: ', num2str(w(2))])
%     disp(['Calc wd: ', num2str(c(4,i))])
% %     disp('Calc Rd: ')
% %     disp((quat2rotm(q_n) - quat2rotm(q_l))/dt)
% %     disp('Diff Rd: ')
% %     disp(Rd)
%     disp('Rot w: ')
%     disp(theta_d)
%     disp('Rot wd: ')
%     disp(theta_dd)
%     disp('Rot w2: ')
%     disp(theta_d2)
%     disp('Rot wd2: ')
%     disp(theta_dd2)
%     disp('---')
% 
%     if i > 1500
%         error('done')
%     end


    % Simulte time step
    %x(:,i) = x(:,i-1) + spline_sim_run(x(:,i-1),c(4,i))*dt;

	% Angular derivative mapping/controller testing
    %XXX: Need '-u' to invert the angle as pitch directions are
    %     reversed to the X axis frame. This means that theta,
    %     theta_d, and theta_dd equations are all inverted
    %theta = atan2(s(3,i), g);
    c(2,i) = atan2(c(1,i), g);
%     c(3,i) = (g*s(4,i)) / (s(3,i)^2 + g^2); %theta_d
%     c(4,i) = -g*(2*s(3,i)*s(4,i)^2 - (s(3,i)^2 + g^2)*s(5,i)) / ((s(3,i)^2 + g^2)^2); %theta_dd
    c(3,i) = theta_d3(2);
    c(4,i) = theta_dd3(2);
    

    % Calculate acceleration input for CTC
    v = c(4,i) + KtP*(c(2,i) - x(3,i-1)) + KtdP*(c(3,i) - x(4,i-1));
    
    x(:,i) = x(:,i-1) + spline_sim_run(x(:,i-1), v, mass)*dt;
end

disp(' ')

%% Plotting

x_error = s(1,:)-x(1,:);
w_error = c(3,:)-x(4,:);
x_RMSE = sqrt(mean((x_error).^2));  % Root Mean Squared Error
w_RMSE = sqrt(mean((w_error).^2));  % Root Mean Squared Error
disp('Max Error: ')
disp(max(abs(x_error)))
disp('Position RMSE: ')
disp(x_RMSE)
disp('Omega RMSE: ')
disp(w_RMSE)

f1 = figure('Renderer','opengl');
    clf;

    set(f1,'defaultAxesColorOrder',[0,0,0;0,0,0]);

    subplot(5,1,1)
        title(plot_title)

        hold on;
        yyaxis left;
        plot(t,s(1,:), 'r-');
        plot(t,x(1,:), 'b-');
        scatter(t_vias, vias(1,:), 'ro')
        yyaxis right;
        plot(t,sin(linspace(0,2*pi,length(t))), 'k--')
        hold off;

        grid on;
        yyaxis left;
        ylabel('Position (m)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Sinusoudal Referece (m)');
        ylim([-maxlim maxlim]);
    subplot(5,1,2)
        hold on;
        plot(t,s(2,:), 'r-');
        plot(t,x(2,:), 'b-');
        scatter(t_vias, vias(2,:), 'ro')
        hold off;

        grid on;
        ylabel('Velocity (m/s)');
    subplot(5,1,3)
        hold on;
        yyaxis left;
        plot(t,c(2,:), 'r-');
        plot(t,x(3,:), 'b-');
        plot(t,atan2(s(3,:), g), 'g--');
        yyaxis right;
        plot(t,s(3,:), 'k--');
        scatter(t_vias, vias(3,:), 'ko');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch (rad)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Acceleration (m/s^2)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
%         leg = legend('Reference', 'State');
%         set(leg, 'Position',[0.708869049162382 0.242634804985937 0.194642854056188 0.0821428550141199]);

    subplot(5,1,4)
        hold on;
        yyaxis left;
        plot(t,c(3,:), 'r-');
        plot(t,x(4,:), 'b-');
        yyaxis right;
        plot(t,s(4,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch Rate (rad/s)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Jerk (m/s^3)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);

    subplot(5,1,5)
        hold on;
        yyaxis left;
        plot(t,c(4,:), 'r-');
        yyaxis right;
        plot(t,s(5,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch Acceleration (rad/s^2)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Snap (m/s^4)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);

% print(f1, ['./figures',plot_titles{2}], '-depsc')


