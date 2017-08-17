function [ I ] = calc_cylinder( mass, dim )
%CALC_CYLINDER Summary of this function goes here
%   Detailed explanation goes here

    assert(isequal(size(dim), [1,2]), 'Hollow cylinder dim must be: [radius, height]')

    I = zeros(3,3);

	I(1,1) = (mass/12)*(3*(dim(1)^2) + (dim(2)^2));
	I(2,2) = (mass/12)*(3*(dim(1)^2) + (dim(2)^2));
	I(3,3) = (mass/2)*(dim(1)^2);
end

