% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ ew ] = error_w_trans( w_sp, w, R_sp, R )
%ERROR_W_TRANS Summary of this function goes here
%   Detailed explanation goes here

    % Rp -> Rsp
    eR = R'*R_sp;

    % ew is the omega error in the body frame at the current time
    ew = eR*w_sp - w;
end

