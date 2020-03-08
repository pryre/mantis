% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [xd] = spline_mantis_sim_run(x,tau,model)
%PCT_RUN Summary of this function goes here
%   Detailed explanation goes here
% x = [zeros(3,1);    % Angular Velocity (Body)
%      zeros(3,1)];   % Linear Velocity (Body)
% v = zeros(size(x)); % Body frame acclerations
% mass_c % mass as seen by the controller
% mass_d % mass as seen by the dynamics
  
%     % Variables
%     dx = zeros(size(x));
    
    sn = state_names_lookup(model.NB-6);

%     [ Dq, Cqqd ] = calc_mass_properties( masses, n, x(sn.STATE_R), x(sn.STATE_REDUCED) );
%     
%     dx = inv(Dq)*(tau - Cqqd*x(sn.STATE_R));

    % XXX: Might have to use FD() instead, similar to IDfb()/ID()
%     xfb = ... ; % Position and velocity of the base

    [af, qdd] = FDfly( model, ...
                       x(sn.STATE_XI_B), x(sn.STATE_R), x(sn.STATE_RD), ...
                       tau(sn.STATE_REDUCED_XI_B), tau(sn.STATE_REDUCED_RD) );

    xd = [ af; ...
          qdd];
end










