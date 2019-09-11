function [ s ] = sign_no_zero( x )
%SIGN_NO_ZERO Summary of this function goes here
%   Detailed explanation goes here

    s = (zeros(size(x)) <= x) - (x < zeros(size(x)));
end

