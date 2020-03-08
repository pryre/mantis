% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ J ] = jacobian_gen( g, q )
%JACOBIAN_GEN Summary of this function goes here
%   Detailed explanation goes here

    J = sym(zeros(6,numel(q)));
    ginv = inverse_trans(g);

    for i = 1:numel(q)
        Ji = ginv*diff(g,q(i));
        J(:,i) = [Ji(1:3,4); vee_down(Ji(1:3,1:3))];
    end
    
end

