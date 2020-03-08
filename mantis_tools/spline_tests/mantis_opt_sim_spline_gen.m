% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ t, s_x, s_y, s_z, s_psi, s_r ] = mantis_opt_sim_spline_gen( config, vias, num_samples )
%MANTIS_SIM_SPLINE_GEN Summary of this function goes here
%   Detailed explanation goes here

    [t, s_x] = gen_spline_samples(vias.time, vias.x, config.spline.order, num_samples);
    [~, s_y] = gen_spline_samples(vias.time, vias.y, config.spline.order, num_samples);
    [~, s_z] = gen_spline_samples(vias.time, vias.z, config.spline.order, num_samples);
    [~, s_psi] = gen_spline_samples(vias.time, vias.psi, config.spline.order, num_samples);
    s_r = cell(config.model.n,1);
    for i = 1:config.model.n
        [~, s_r{i}] = gen_spline_samples(vias.time, vias.r{i}, config.spline.order, num_samples);
    end
    
end

