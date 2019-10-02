function [ gi ] = inverse_trans( g )
%INVERSE_TRANS Summary of this function goes here
%   Detailed explanation goes here

    gp = g(1:3,4);
    gR = g(1:3,1:3);
    gi = [       gR', -gR'*gp; ...
          zeros(1,3),       1];
end

