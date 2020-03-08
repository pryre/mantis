% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ Rd ] = rd_from_dvec_dyaw( v, dv, yaw_c, dyaw_c )
%QUATERNION_FROM_ACCEL_YAW Summary of this function goes here
%   Detailed explanation goes here

    SIGMA = 0.0001;

    if ~iscolumn(v)
        v = v';
    end

    r_x = zeros(3,1);
    r_y = zeros(3,1);
    r_z = zeros(3,1);
    rd_x = zeros(3,1);
    rd_y = zeros(3,1);
    rd_z = zeros(3,1);

    % If we have a reasonable vector length
    if norm(v) > SIGMA
        r_z = v/norm(v);
        rd_z = vd/norm(vd);
    else
        warning('Small vector detected! Assuming hold upright.');
        r_z(3) = 1;
        rd_z(3) = 1;
    end

    % If there is at least some vertical aspect (non-lateral vector)
    if abs( r_z(3) ) > SIGMA
        % Get a vector that aligns the Y axis with goal yaw
        yaw_r = [-sin( yaw_c ), cos( yaw_c ), 0.0 ];
        dyaw_r = [-dyaw_c*cos( yaw_c ), -dyaw_c*sin( yaw_c ), 0.0 ];

        % Get the orthagonal vector to that (which will have correct pitch)
        r_x = cross( yaw_r, r_z );

        % Inverted flight, keep nose in right direction
        % while inverted upside down
        if r_z(3) < 0.0
            r_x = -r_x;
        end

        % Make sure we have a normalised vector
        r_x = r_x/norm(r_x);
    else
        % Desired thrust is in XY plane, set X downwards to construct
        % correct matrix (yaw component will not be used)
        r_x(3) = 1;
        warning( "Lateral thrust command, ignoring yaw!" );
    end

    % Align the Y axis to be orthoganal with XZ plane,
    % cross product should already be close enough to normalised
    r_y = cross(r_z, r_x);

    % Construct rotation matrix
    % (dot expansion forces column vectors)
    R = [r_x(:), r_y(:), r_z(:)];

    % Get quaternion representation
    q = rotm2quat(R);
end

