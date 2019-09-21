function [ m ] = calc_total_mass( model )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    m = 0;
    for i = 6:model.NB
        m = m + model.I{i}(6,6);
    end
end

