function [ I ] = calc_sphere( mass, dim )
%CALC_SPHERE Summary of this function goes here
%   Detailed explanation goes here

    assert(isequal(size(dim), [1,1]), 'Sphere dim must be: [radius]')

    I = zeros(3,3);

	Ixyz = (2*mass/5)*(dim*dim);
    I(1,1) = Ixyz;
    I(2,2) = Ixyz;
    I(3,3) = Ixyz;
end

