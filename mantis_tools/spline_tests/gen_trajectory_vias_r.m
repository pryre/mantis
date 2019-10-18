function [via_points_r ] = gen_trajectory_vias_r(trajectory,num_via_points)
%TRAJECTORY_VIAS Summary of this function goes here
%   Detailed explanation goes here

    if strcmp(trajectory,'steady_0')
        via_points_r = zeros(1,num_via_points);
    elseif strcmp(trajectory,'steady_90')
        via_points_r = (pi/2)*ones(1,num_via_points);
    elseif strcmp(trajectory,'swing_part')
        via_points_r = linspace(0,pi/4,num_via_points);
    elseif strcmp(trajectory,'swing_half')
        via_points_r = linspace(0,pi/2,num_via_points);
    elseif strcmp(trajectory,'swing_full')
        via_points_r = linspace(-pi/2,pi/2,num_via_points);
    else
        error(['Unknown joint trajectory: ', trajectory])
    end
end

