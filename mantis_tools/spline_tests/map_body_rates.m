function [ w ] = map_body_rates( a, ad )
%UNTITLED2 Summary of this function goes here
%   a:   Acceleration
%   ad:  Jerk
%   add: Snap

    % a_n is the normalized acceleration vector
    % ad_t is the jerk component that is tangental to acceleration vector
    a_n = a / norm(a);
    ad_t = ad - dot(ad,a_n)*a_n; 
    w = cross(a,ad_t) / (norm(a)^2);
end

