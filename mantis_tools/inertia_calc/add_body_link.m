function [ body ] = add_body_link( body, name, type, mass, center_of_mass, rotation, dim )
%ADD_BODY_LINK Summary of this function goes here
%   Detailed explanation goes here

    body(end+1,:) = {name, type, mass, center_of_mass, rotation, dim, zeros(3,3)};

end

