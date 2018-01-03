function [ ad ] = adjoint_trans( g )
%ADJOINT The 6x6 matrix which transforms twists by the spacial
%transformation of g
%   Detailed explanation goes here

    gR = g(1:3,1:3);
    gph = vee_up(g(1:3,4));

    ad = [      gR, gph*gR; ...
          zeros(3),     gR];

end

