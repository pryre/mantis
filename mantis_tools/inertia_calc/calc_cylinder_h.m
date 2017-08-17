function [ I ] = calc_cylinder_h( mass, dim )
%CALC_CYLINDER_H Summary of this function goes here
%   Detailed explanation goes here

    assert(isequal(size(dim), [1,3]), 'Hollow cylinder dim must be: [inner_radius, outer_radius, height]')

    I = zeros(3,3);

	I(1,1) = (mass/12)*(3*((dim(1)^2)+(dim(2)^2)) + (dim(3)^2));
	I(2,2) = (mass/12)*(3*((dim(1)^2)+(dim(2)^2)) + (dim(3)^2));
	I(3,3) = (mass/2)*((dim(1)^2) + (dim(2)^2));
end

