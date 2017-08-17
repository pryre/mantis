function [ I ] = calc_sphere_h( mass, dim )
%CALC_SPHERE_H Summary of this function goes here
%   Detailed explanation goes here

    assert(isequal(size(dim), [1,1]), 'Hollow sphere dim must be: [radius]')
    
    I = zeros(3,3);

	Ixyz = (2*mass/3)*(dim*dim);
    I(1,1) = Ixyz;
    I(2,2) = Ixyz;
    I(3,3) = Ixyz;
end

