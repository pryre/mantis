function [ I ] = calc_ellipsoid( mass, dim )
%CALC_ELLIPSOID Summary of this function goes here
%   Detailed explanation goes here

    assert(isequal(size(dim), [1,3]), 'Ellipsoid dim must be: [x, y, z]')

    I = zeros(3,3);

	I(1,1) = (mass/5)*((dim(2)^2) + (dim(3)^2));
	I(2,2) = (mass/5)*((dim(1)^2) + (dim(3)^2));
	I(3,3) = (mass/5)*((dim(1)^2) + (dim(2)^2));
end

