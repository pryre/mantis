function [ M ] = spline_solver_gen_tnorm_ls( dim )
%SPLINE_SOLVER_GEN_LINEAR_SYSTEM Generates a linear system of a set
%dimension for the use in solving for a spline in the case that the spline 
%is time-normalised.
%   Inputs:
%       dim: Should be the dimension of the linear system, typically should be
%            spline order + 1
%   Outputs:
%         M: The generated time-normalised linear system

    if ~isscalar(dim)
        error(['dim must a scalar number: ', num2str(dim)])
    end
    
    if mod(dim,2) || dim <= 0
        error(['dim must be a positive even number: ', num2str(dim)])
    end
    
    M = zeros(dim,dim);

    % Go through all columns
    for i = 1:dim
        order = i-1;
        coeff = 1;
        
        % Go through both sets of t0 and tf simultaneously
        for j = 1:dim/2
            if order >= 0
                % Fill in t0 section
                M(j,i) = coeff*(0^order);
                % Fill in tf section
                M((j+dim/2),i) = coeff*(1^order);
                
                % "Derivation" step
                coeff = coeff*order;
                order = order - 1;
            end % Else M(j,i) == 0, but already initialised to 0
        end
    end
end

