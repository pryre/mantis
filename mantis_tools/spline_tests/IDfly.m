% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function  [ tauf, tau ] = IDfly( model, vf, af, q, qd, qdd, f_ext )

    % TODO: Update this description!
    % ID  Inverse Dynamics via Recursive Newton-Euler Algorithm
    % ID(model,q,qd,qdd,f_ext) calculates the inverse dynamics of a kinematic
    % tree via the recursive Newton-Euler algorithm.  q, qd and qdd are vectors
    % of joint position, velocity and acceleration variables; and the return
    % value is a vector of joint force variables.  f_ext is an optional
    % argument specifying the external forces acting on the bodies.  It can be
    % omitted if there are no external forces.  The format of f_ext is
    % explained in the source code of apply_external_forces.

    % We don't apply gravity in this model, assumed to be free-flying
    %     a_grav = get_gravity(model);

    % Set the accelerations and velocities of the base in the base frame 
    v{6} = vf;
    a{6} = af;

    % Apply the forces of the base acting on the base
    % H11 -> ST1*IC1*S1 -> IC1
    f{6} = model.I{6}*a{6} + crf(v{6})*model.I{6}*v{6}; 

    for i = 7:model.NB
        j = i-6;
        [ XJ, S{i} ] = jcalc( model.jtype{i}, q(j) );
        vJ = S{i}*qd(j);
        Xup{i} = XJ * model.Xtree{i};
        % Calculate the accelerations and velocites acting on each joint in
        % the joint frame, adding in the effects from the parent bodies
        v{i} = Xup{i}*v{model.parent(i)} + vJ;
        a{i} = Xup{i}*a{model.parent(i)} + S{i}*qdd(j) + crm(v{i})*vJ;
        % Calculate the forces necessary to perform these actions
        f{i} = model.I{i}*a{i} + crf(v{i})*model.I{i}*v{i};
    end

    % Apply external body forces (if requried) 
    if nargin == 7
        apply_external_forces( model.parent, Xup, f, f_ext )
    end

    for i = model.NB:-1:7
        j = i-6;
        % Calculate the joint-space forces required 
        tau(j,1) = S{i}' * f{i};
        % Propogate the joint forces from the end effectors back through
        % each parent link (up until the free-flying base)
        f{model.parent(i)} = f{model.parent(i)} + Xup{i}'*f{i};
    end
    
    % Get the final forces acting on the free-flying base after propogation
    tauf = f{6};
end
