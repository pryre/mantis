% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [via_points_x, via_points_y, via_points_z, via_points_psi ] = gen_trajectory_vias(trajectory,num_via_points)
%TRAJECTORY_VIAS Summary of this function goes here
%   Detailed explanation goes here

    via_points_x = ones(1,num_via_points);
    via_points_y = zeros(1,num_via_points);
    via_points_z = ones(1,num_via_points);
    via_points_psi = zeros(1,num_via_points);
    
    x_traj = cos(linspace(0,2*pi,num_via_points));
    y_traj = sin(linspace(0,2*pi,num_via_points));
    z_traj = linspace(1,4,num_via_points);
    psi_traj = linspace(0,2*pi,num_via_points);

    if strcmp(trajectory,'hover')
        via_points_x = via_points_y;
    elseif strcmp(trajectory,'hover2')
        via_points_x = via_points_y;
        via_points_z = 2*ones(1,num_via_points);
    elseif strcmp(trajectory,'x_only')
        via_points_x = x_traj;
    elseif strcmp(trajectory,'x_only_1m')
        via_points_x = linspace(0,1,num_via_points);
    elseif strcmp(trajectory,'y_only')
        via_points_y = y_traj;
    elseif strcmp(trajectory,'z_only')
        via_points_z = z_traj;
    elseif strcmp(trajectory,'yaw_only')
        via_points_psi = psi_traj;
    elseif strcmp(trajectory,'x_yaw')
        via_points_x = x_traj;
        via_points_psi = psi_traj;
    elseif strcmp(trajectory,'y_yaw')
        via_points_y = y_traj;
        via_points_psi = psi_traj;
    elseif strcmp(trajectory,'z_yaw')
        via_points_z = z_traj;
        via_points_psi = psi_traj;
    elseif strcmp(trajectory,'circle_flat')
        via_points_x = x_traj;
        via_points_y = y_traj;
    elseif strcmp(trajectory,'circle_flat_yaw')
        via_points_x = x_traj;
        via_points_y = y_traj;
        via_points_psi = psi_traj;
    elseif strcmp(trajectory,'circle_raised')
        via_points_x = x_traj;
        via_points_y = y_traj;
        via_points_z = z_traj;
    elseif strcmp(trajectory,'circle_raised_yaw')
        via_points_x = x_traj;
        via_points_y = y_traj;
        via_points_z = z_traj;
        via_points_psi = psi_traj;
    else
        error(['Unknown trajectory: ', trajectory])
    end
end

